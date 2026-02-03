import { NextRequest, NextResponse } from "next/server";
import path from "path";
import { promises as fs } from "fs";

const UPLOAD_ROOT = path.join(process.cwd(), "public", "uploads");

export async function GET(request: NextRequest) {
  try {
    const pathname = request.nextUrl.pathname;
    const relative = pathname.split("/api/uploads/")[1] ?? "";
    const segments = relative.split("/").filter(Boolean);
    if (segments.length === 0) {
      return NextResponse.json({ error: "Missing file path" }, { status: 400 });
    }
    const safePath = path.join(UPLOAD_ROOT, ...segments);
    if (!safePath.startsWith(UPLOAD_ROOT)) {
      return NextResponse.json({ error: "Invalid path" }, { status: 403 });
    }
    const data = await fs.readFile(safePath);
    const ext = path.extname(safePath).toLowerCase();
    const contentType = ext === ".json"
      ? "application/json"
      : "application/octet-stream";
    const body = new Uint8Array(data as Buffer);
    return new NextResponse(body, {
      status: 200,
      headers: {
        "Content-Type": contentType,
        "Cache-Control": "public, max-age=604800, immutable",
      },
    });
  } catch (error) {
    console.error("Failed to read upload", error);
    return NextResponse.json({ error: "Not found" }, { status: 404 });
  }
}
