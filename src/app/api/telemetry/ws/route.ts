import { NextRequest } from "next/server";

// minimal type for Edge runtime WebSocketPair
declare const WebSocketPair: {
  new (): { 0: WebSocket; 1: WebSocket };
};

export const runtime = "edge";

function toStream(
  controller: ReadableStreamDefaultController,
  simulate = false
) {
  let timer: number | undefined;
  if (simulate) {
    let t = 0;
    timer = self.setInterval(() => {
      // simple circle path
      t += 0.1;
      const r = 1.5;
      const x = Math.cos(t) * r;
      const y = Math.sin(t) * r;
      const z = Math.sin(t * 0.5) * 0.5;
      controller.enqueue(
        new TextEncoder().encode(
          JSON.stringify({ x, y, z, t: Date.now() }) + "\n"
        )
      );
    }, 100);
  }
  return () => {
    if (timer) self.clearInterval(timer);
  };
}

export async function GET(req: NextRequest) {
  // Implement a simple WebSocket over HTTP upgrade using WebSocketPair (Edge)
  const { searchParams } = new URL(req.url);
  const simulate = searchParams.get("simulate") === "1";

  const upgradeHeader = req.headers.get("upgrade") || "";
  if (upgradeHeader.toLowerCase() !== "websocket") {
    return new Response("Expected WebSocket", { status: 426 });
  }

  const { 0: client, 1: server } = new WebSocketPair();
  // Edge runtime requires accept()
  // @ts-expect-error accept is available in Edge runtime
  server.accept();

  let cleanup: (() => void) | undefined;

  if (simulate) {
    // stream timer that emits points, forward as WS messages
    const stream = new ReadableStream({
      start(controller) {
        cleanup = toStream(controller, true);
      },
      cancel() {
        cleanup?.();
      },
    });
    const reader = stream.getReader();
    (async () => {
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) {
            // forward to client
            server.send(new TextDecoder().decode(value));
          }
        }
      } catch {}
    })();
  }

  server.addEventListener("message", (ev: MessageEvent) => {
    // echo back or process
    server.send(ev.data);
  });
  server.addEventListener("close", () => cleanup?.());

  return new Response(null, {
    status: 101,
    webSocket: client,
  } as unknown as ResponseInit);
}
