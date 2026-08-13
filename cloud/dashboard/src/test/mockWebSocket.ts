/**
 * A minimal fake `WebSocket` for tests -- no real network, but the exact same event-handler
 * surface (`onopen`/`onmessage`/`onclose`/`onerror`, `.close()`) `useLiveSocket.ts` actually uses,
 * so a test can drive it by calling `instance.emitMessage(rawJsonString)` directly rather than
 * needing a real server. `MockWebSocket.instances` lets a test grab whichever instance the
 * component under test constructed (there's exactly one per connection attempt).
 */
export class MockWebSocket {
  static instances: MockWebSocket[] = [];

  url: string;
  onopen: (() => void) | null = null;
  onmessage: ((event: { data: string }) => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: (() => void) | null = null;
  closed = false;

  constructor(url: string) {
    this.url = url;
    MockWebSocket.instances.push(this);
  }

  emitOpen() {
    this.onopen?.();
  }

  emitMessage(raw: unknown) {
    const data = typeof raw === "string" ? raw : JSON.stringify(raw);
    this.onmessage?.({ data });
  }

  close() {
    this.closed = true;
    this.onclose?.();
  }

  static reset() {
    MockWebSocket.instances = [];
  }

  static latest(): MockWebSocket {
    const instance = MockWebSocket.instances[MockWebSocket.instances.length - 1];
    if (!instance) throw new Error("No MockWebSocket instance was constructed yet.");
    return instance;
  }
}
