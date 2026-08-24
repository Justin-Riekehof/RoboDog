"""The intent client, against a fake OpenAI-compatible server in this process.

Nothing here talks to the owner's vLLM box: the server below records what was
asked and answers what the test wants, which is how the awkward answers -- a
model that reasoned instead of answering, one that emitted a behaviour that does
not exist, one that refuses the thinking switch -- become ordinary tests.

The numbers in the module docstring of robodog.ai came from the real server;
these tests pin the behaviour those numbers led to.
"""

from __future__ import annotations

import json
import threading
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import pytest

from robodog.ai import ENV_URL, IntentClient, LlmConfig
from robodog.behaviour import VOCABULARY, call_schema, parse_call, system_prompt, validate_call
from robodog.errors import AiError, BehaviourError


class FakeLlm:
    """An OpenAI-compatible endpoint with a scripted answer."""

    def __init__(self) -> None:
        self.requests: list[dict[str, Any]] = []
        self.models = ["fake-qwen", "another"]
        self.content: str | None = '{"behaviour": "come_to_me", "target": "person"}'
        self.finish_reason = "stop"
        self.status = 200
        self.error_body = ""
        self.refuse_thinking_switch = False

    def completion(self) -> dict[str, Any]:
        return {
            "choices": [
                {
                    "index": 0,
                    "message": {"role": "assistant", "content": self.content},
                    "finish_reason": self.finish_reason,
                }
            ]
        }


def serve(fake: FakeLlm) -> Iterator[str]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: object) -> None:
            pass

        def _send(self, status: int, payload: object) -> None:
            body = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path.endswith("/models"):
                self._send(200, {"data": [{"id": name} for name in fake.models]})
            else:
                self._send(404, {"error": "no"})

        def do_POST(self) -> None:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            fake.requests.append(body)
            if fake.refuse_thinking_switch and "chat_template_kwargs" in body:
                self._send(400, {"error": "unexpected field chat_template_kwargs"})
                return
            if fake.status != 200:
                self._send(fake.status, {"error": fake.error_body})
                return
            self._send(200, fake.completion())

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    # A short poll interval so shutdown() returns at once: at the default
    # half second, a file of these tests spends most of its time in teardown.
    threading.Thread(
        target=httpd.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True
    ).start()
    try:
        yield f"http://127.0.0.1:{httpd.server_address[1]}/v1"
    finally:
        httpd.shutdown()
        httpd.server_close()


@pytest.fixture
def llm() -> Iterator[tuple[FakeLlm, IntentClient]]:
    fake = FakeLlm()
    for url in serve(fake):
        yield fake, IntentClient(LlmConfig(base_url=url))


# --- the request ------------------------------------------------------------


def test_thinking_is_off_by_default(llm: tuple[FakeLlm, IntentClient]) -> None:
    """Measured on the owner's server: 30.8 s of reasoning against 0.68 s
    without it, for the same answer. There is nothing here to reason about."""
    fake, client = llm
    client.interpret("Komm zu mir")
    assert fake.requests[0]["chat_template_kwargs"] == {"enable_thinking": False}


def test_the_answer_is_constrained_to_the_vocabulary_schema(
    llm: tuple[FakeLlm, IntentClient],
) -> None:
    fake, client = llm
    client.interpret("Komm zu mir")
    schema = fake.requests[0]["response_format"]["json_schema"]["schema"]
    assert schema == call_schema()
    assert schema["properties"]["behaviour"]["enum"] == list(VOCABULARY)


def test_the_prompt_carries_the_same_vocabulary(llm: tuple[FakeLlm, IntentClient]) -> None:
    fake, client = llm
    client.interpret("Komm zu mir")
    prompt = fake.requests[0]["messages"][0]["content"]
    assert prompt == system_prompt()
    for name in VOCABULARY:
        assert name in prompt


def test_the_model_is_discovered_when_it_was_not_configured(
    llm: tuple[FakeLlm, IntentClient],
) -> None:
    fake, client = llm
    assert client.model() == "fake-qwen"
    client.interpret("Komm zu mir")
    assert fake.requests[0]["model"] == "fake-qwen"


def test_a_configured_model_is_used_without_asking() -> None:
    client = IntentClient(LlmConfig(base_url="http://127.0.0.1:1/v1", model="named"))
    assert client.model() == "named"  # no request needed, so the dead URL is fine


# --- the answer -------------------------------------------------------------


def test_a_plain_answer_becomes_a_call(llm: tuple[FakeLlm, IntentClient]) -> None:
    _fake, client = llm
    call = client.interpret("Komm zu mir")
    assert call.name == "come_to_me"
    assert call.params["target"] == "person"
    assert call.understood


def test_a_refusal_is_an_answer_not_an_error(llm: tuple[FakeLlm, IntentClient]) -> None:
    """A model with no way to say 'not in my vocabulary' always improvises one."""
    fake, client = llm
    fake.content = '{"behaviour": "unknown"}'
    call = client.interpret("mach einen Rueckwaertssalto")
    assert not call.understood


def test_a_think_block_is_stripped_from_the_content(llm: tuple[FakeLlm, IntentClient]) -> None:
    fake, client = llm
    fake.content = '<think>hmm, German for come here</think>\n{"behaviour": "stop"}'
    assert client.interpret("halt").name == "stop"


def test_a_model_that_only_reasoned_is_reported_as_that(
    llm: tuple[FakeLlm, IntentClient],
) -> None:
    """The failure this module exists to avoid, named where the fix is."""
    fake, client = llm
    fake.content = ""
    fake.finish_reason = "length"
    with pytest.raises(AiError, match="whole 256-token budget"):
        client.interpret("Komm zu mir")


def test_a_behaviour_outside_the_vocabulary_is_refused(
    llm: tuple[FakeLlm, IntentClient],
) -> None:
    fake, client = llm
    fake.content = '{"behaviour": "backflip"}'
    with pytest.raises(AiError, match="unknown behaviour"):
        client.interpret("backflip")


def test_an_answer_that_is_not_json_is_refused(llm: tuple[FakeLlm, IntentClient]) -> None:
    fake, client = llm
    fake.content = "Sure! I'll come over."
    with pytest.raises(AiError, match="did not answer with JSON"):
        client.interpret("Komm zu mir")


# --- the server misbehaving -------------------------------------------------


def test_an_http_error_carries_what_the_server_said(
    llm: tuple[FakeLlm, IntentClient],
) -> None:
    fake, client = llm
    fake.status = 500
    fake.error_body = "engine died"
    with pytest.raises(AiError, match="HTTP 500"):
        client.interpret("Komm zu mir")


def test_a_server_that_refuses_the_thinking_switch_is_asked_again_without_it(
    llm: tuple[FakeLlm, IntentClient],
) -> None:
    """Losing the speed is better than losing the ability to ask at all."""
    fake, client = llm
    fake.refuse_thinking_switch = True
    call = client.interpret("Komm zu mir")
    assert call.name == "come_to_me"
    assert len(fake.requests) == 2
    assert "chat_template_kwargs" not in fake.requests[1]
    # With room to think, or the retry fails again for a different reason.
    assert fake.requests[1]["max_tokens"] > fake.requests[0]["max_tokens"]


def test_an_unreachable_server_says_where_to_look() -> None:
    client = IntentClient(LlmConfig(base_url="http://127.0.0.1:1/v1", model="x", timeout=0.3))
    with pytest.raises(AiError, match=ENV_URL):
        client.interpret("Komm zu mir")


def test_nothing_said_is_refused_before_any_request() -> None:
    client = IntentClient(LlmConfig(base_url="http://127.0.0.1:1/v1", model="x"))
    with pytest.raises(AiError, match="nothing was said"):
        client.interpret("   ")


# --- the vocabulary itself --------------------------------------------------


def test_every_behaviour_in_the_schema_can_be_validated() -> None:
    """One table, three consumers: schema, prompt and validator cannot drift."""
    for name in call_schema()["properties"]["behaviour"]["enum"]:
        assert validate_call(name).name == name


def test_a_parameter_for_another_behaviour_is_dropped_not_fatal() -> None:
    call = validate_call("stop", {"target": "person"})
    assert call.params == {}


def test_a_choice_outside_the_list_is_refused() -> None:
    with pytest.raises(BehaviourError, match="is not one of"):
        validate_call("come_to_me", {"target": "dragon"})


def test_a_parameter_of_the_wrong_type_is_refused() -> None:
    with pytest.raises(BehaviourError, match="must be a number"):
        validate_call("come_to_me", {"stop_distance_mm": "far"})


def test_parse_call_rejects_a_non_object_answer() -> None:
    with pytest.raises(BehaviourError, match="not an object"):
        parse_call("[1, 2, 3]")
