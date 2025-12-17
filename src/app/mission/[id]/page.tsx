import MissionDetailClient from "./MissionDetailClient";

type MissionDetailPageProps = {
  params: Promise<{ id: string }>;
};

export default async function MissionDetailPage({ params }: MissionDetailPageProps) {
  const { id } = await params;
  return <MissionDetailClient missionId={id} />;
}
