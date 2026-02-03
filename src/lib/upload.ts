export async function uploadMissionFile(file: File, category: 'scene' | 'trajectory'): Promise<string> {
  const formData = new FormData();
  formData.append('category', category);
  formData.append('file', file);

  const response = await fetch('/api/uploads', {
    method: 'POST',
    body: formData,
  });

  if (!response.ok) {
    throw new Error('文件上传失败');
  }

  const data = await response.json();
  if (!data?.url) {
    throw new Error('服务器未返回文件地址');
  }
  return data.url as string;
}
