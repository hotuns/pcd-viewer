import { NextRequest, NextResponse } from "next/server";
import path from "path";
import { promises as fs } from "fs";

const CATEGORY_DIRS: Record<string, string> = {
  scene: "scenes",
  trajectory: "trajectories",
};

const UPLOAD_ROOT = path.join(process.cwd(), "public", "uploads");

async function ensureDir(dir: string) {
  try {
    await fs.mkdir(dir, { recursive: true });
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code !== "EEXIST") {
      throw error;
    }
  }
}

export async function POST(request: NextRequest) {
  try {
    const formData = await request.formData();
    const category = (formData.get("category") as string) || "scene";
    const file = formData.get("file");

    if (!(file instanceof File)) {
      return NextResponse.json({ error: "Missing file" }, { status: 400 });
    }

    const subdir = CATEGORY_DIRS[category] ?? CATEGORY_DIRS.scene;
    const targetDir = path.join(UPLOAD_ROOT, subdir);
    await ensureDir(targetDir);

    const arrayBuffer = await file.arrayBuffer();
    const buffer = Buffer.from(arrayBuffer);
    const originalExt = path.extname(file.name) || (category === "trajectory" ? ".json" : ".pcd");
    const filename = `${Date.now()}_${Math.random().toString(36).slice(2)}${originalExt}`;
    const filePath = path.join(targetDir, filename);
    await fs.writeFile(filePath, buffer);

    const apiUrl = `/api/uploads/${subdir}/${filename}`;
    return NextResponse.json({ url: apiUrl });
  } catch (error) {
    console.error("Failed to upload file", error);
    return NextResponse.json({ error: "Failed to upload file" }, { status: 500 });
  }
}
