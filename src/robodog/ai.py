"""Free text to a behaviour call, against an OpenAI-compatible endpoint.

The owner runs a vLLM server; the model is a large one and the request below is
tiny, because of where it sits in the system: **the model does not drive.** It
is asked once, before anything moves, to map what the operator said onto one
entry of :mod:`robodog.behaviour.vocabulary`. A deterministic loop then runs
that behaviour through ``SafetySupervisor`` like every other command path.

Two reasons, and the first is this repo's hard rule: nothing reaches a backend
around the supervisor. The second is that a language model inside a control
loop is latency and non-determinism in the one place neither belongs.

Three things were measured against the owner's server (vLLM 0.27.1, Qwen3.8-27B
FP8, 2026-08-23) and each shaped this module:

* **Thinking is switched off**, via ``chat_template_kwargs``. The same request
  took **30.8 s** with reasoning enabled and **0.68 s** without it, for an
  identical answer -- 1225 completion tokens against 23. For a one-shot mapping
  onto a three-entry vocabulary there is nothing to reason about, and half a
  minute before the robot moves is not a UI.
* **The answer is grammar-constrained**, via ``response_format`` with the
  schema the vocabulary generates. The model cannot emit a behaviour that does
  not exist; it can only choose badly, and ``unknown`` is one of the choices.
* **Reasoning still has to be handled** even so, because a server may ignore
  the switch: the reply is read from ``content``, ``<think>`` blocks are
  stripped, and a reply that spent its whole budget thinking is reported as
  that rather than as a parse error.

No dependency: urllib, like :mod:`robodog.backends.http`. An API key is
optional and none is assumed.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass, replace
from typing import Any, Final

from robodog.behaviour.vocabulary import BehaviourCall, call_schema, parse_call, system_prompt
from robodog.errors import AiError, BehaviourError

DEFAULT_BASE_URL: Final = "http://127.0.0.1:8000/v1"
DEFAULT_TIMEOUT: Final = 30.0
# Enough for the JSON and nothing like enough for a chain of thought, which is
# deliberate: a server that ignores the thinking switch runs out of budget and
# says so, instead of quietly costing the operator half a minute per command.
DEFAULT_MAX_TOKENS: Final = 256

ENV_URL: Final = "ROBODOG_LLM_URL"
ENV_MODEL: Final = "ROBODOG_LLM_MODEL"
ENV_KEY: Final = "ROBODOG_LLM_KEY"

_THINK_BLOCK: Final = re.compile(r"<think>.*?</think>", re.DOTALL)


@dataclass(frozen=True, slots=True)
class LlmConfig:
    """Where the model is and how it is asked. All of it overridable."""

    base_url: str = DEFAULT_BASE_URL
    # Empty means "whatever the server serves": vLLM is asked for its model
    # list and the first entry is used. One less thing to configure, and one
    # less way for a renamed alias to break a session.
    model: str = ""
    api_key: str | None = None
    timeout: float = DEFAULT_TIMEOUT
    temperature: float = 0.0
    max_tokens: int = DEFAULT_MAX_TOKENS
    thinking: bool = False

    @classmethod
    def from_env(cls, **overrides: Any) -> LlmConfig:
        config = cls(
            base_url=os.environ.get(ENV_URL) or DEFAULT_BASE_URL,
            model=os.environ.get(ENV_MODEL) or "",
            api_key=os.environ.get(ENV_KEY) or None,
        )
        return replace(config, **{k: v for k, v in overrides.items() if v is not None})

    @property
    def url(self) -> str:
        return self.base_url.rstrip("/")


class IntentClient:
    """One request per operator utterance. Holds nothing but its configuration."""

    def __init__(
        self,
        config: LlmConfig | None = None,
        *,
        opener: urllib.request.OpenerDirector | None = None,
    ) -> None:
        self.config = config if config is not None else LlmConfig()
        self._opener = opener or urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._model: str | None = self.config.model or None
        self.last_answer: str = ""

    # --- transport --------------------------------------------------------

    def _request(self, path: str, payload: dict[str, Any] | None) -> dict[str, Any]:
        url = f"{self.config.url}{path}"
        headers = {"Content-Type": "application/json"}
        if self.config.api_key:
            headers["Authorization"] = f"Bearer {self.config.api_key}"
        data = json.dumps(payload).encode("utf-8") if payload is not None else None
        request = urllib.request.Request(url, data=data, headers=headers)
        try:
            with self._opener.open(request, timeout=self.config.timeout) as response:
                body = response.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read()[:400].decode("utf-8", "replace")
            raise AiError(f"{url} returned HTTP {exc.code}: {detail}") from exc
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise AiError(
                f"cannot reach the language model at {url}: {exc} -- "
                f"is the server running, and is {ENV_URL} right?"
            ) from exc
        try:
            parsed: Any = json.loads(body)
        except ValueError as exc:
            raise AiError(f"{url} did not answer with JSON") from exc
        if not isinstance(parsed, dict):
            raise AiError(f"{url} answered with {type(parsed).__name__}, not an object")
        return parsed

    def models(self) -> tuple[str, ...]:
        """What the server serves. Also the cheapest reachability check there is."""
        data = self._request("/models", None).get("data")
        if not isinstance(data, list):
            return ()
        return tuple(
            str(entry["id"]) for entry in data if isinstance(entry, dict) and "id" in entry
        )

    def model(self) -> str:
        """The model this client will use, asking the server if it was not told."""
        if self._model is None:
            served = self.models()
            if not served:
                raise AiError(f"{self.config.url} serves no models")
            self._model = served[0]
        return self._model

    # --- the one thing this class is for ----------------------------------

    def interpret(self, text: str) -> BehaviourCall:
        """Map what the operator said onto one behaviour call.

        Raises AiError when the server cannot be reached or answers with
        something outside the vocabulary. A refusal is *not* an error: it comes
        back as the ``unknown`` behaviour, which the caller reports and does not
        run.
        """
        said = text.strip()
        if not said:
            raise AiError("nothing was said")
        payload = self._payload(said, thinking=self.config.thinking)
        try:
            answer = self._chat(payload)
        except AiError as exc:
            # A server that has never heard of the thinking switch refuses the
            # whole request. Losing the ability to ask is worth more than losing
            # the speed, so try once more without it -- with room to think, or
            # the retry fails a second time for a different reason.
            if not self.config.thinking and "chat_template_kwargs" in str(exc):
                answer = self._chat(self._payload(said, thinking=True, max_tokens=2048))
            else:
                raise
        self.last_answer = answer
        try:
            return parse_call(answer)
        except BehaviourError as exc:
            raise AiError(str(exc)) from exc

    def _payload(
        self, said: str, *, thinking: bool, max_tokens: int | None = None
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "model": self.model(),
            "messages": [
                {"role": "system", "content": system_prompt()},
                {"role": "user", "content": said},
            ],
            "temperature": self.config.temperature,
            "max_tokens": max_tokens if max_tokens is not None else self.config.max_tokens,
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "behaviour_call",
                    "schema": call_schema(),
                    "strict": True,
                },
            },
        }
        if not thinking:
            payload["chat_template_kwargs"] = {"enable_thinking": False}
        return payload

    def _chat(self, payload: dict[str, Any]) -> str:
        data = self._request("/chat/completions", payload)
        choices = data.get("choices")
        if not isinstance(choices, list) or not choices:
            raise AiError("the model returned no choices")
        choice = choices[0]
        message = choice.get("message") if isinstance(choice, dict) else None
        content = (message or {}).get("content") or ""
        if not isinstance(content, str):
            raise AiError(f"the model returned {type(content).__name__} content")
        content = _THINK_BLOCK.sub("", content).strip()
        if content:
            return content
        # Empty content has exactly one interesting cause worth naming, because
        # the fix is a setting rather than a bug hunt.
        if choice.get("finish_reason") == "length":
            raise AiError(
                f"the model used its whole {payload['max_tokens']}-token budget without "
                f"answering -- it is reasoning. Switch thinking off on the server, or "
                f"raise the budget"
            )
        raise AiError("the model answered with nothing")
