import time
import numpy as np
import open3d as o3d

class LSAVisionPipeline:
    def __init__(self, voxel_size=0.01, normal_radius=0.05, max_nn=30):
        # NOTA: Baixei o voxel_size para 1cm (0.01) porque a aranha está a apenas
        # 15cm do tubo (ALTURA = 0.15) e o tubo é pequeno.
        self.voxel_size = voxel_size
        self.normal_radius = normal_radius
        self.max_nn = max_nn

    def processar_nuvem(self, pontos_brutos, cores_brutas, view_matrix):
        """Aplica Voxel Grid e Estimação de Normais à nuvem bruta."""
        t_start = time.perf_counter()

        # 1. Converte NumPy para Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pontos_brutos)
        if cores_brutas is not None:
            pcd.colors = o3d.utility.Vector3dVector(cores_brutas)

        num_pontos_bruto = len(pcd.points)

        # 2. Voxel Grid Filter (Downsampling)
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        num_pontos_down = len(pcd_down.points)

        # 3. Estimação de Normais (usando KD-Tree)
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius, max_nn=self.max_nn
            )
        )

        # 4. Alinha as normais para apontarem para a câmara (lendo a view_matrix)
        cam_position = np.linalg.inv(np.array(view_matrix).reshape((4, 4), order='F'))[:3, 3]
        pcd_down.orient_normals_towards_camera_location(camera_location=cam_position)

        # 5. Telemetria
        tempo_ms = (time.perf_counter() - t_start) * 1000.0
        taxa_comp = (1.0 - (num_pontos_down / num_pontos_bruto)) * 100.0 if num_pontos_bruto > 0 else 0

        estatisticas = {
            "tempo_ms": tempo_ms,
            "pontos_brutos": num_pontos_bruto,
            "pontos_down": num_pontos_down,
            "compressao_pct": taxa_comp
        }

        return pcd_down, estatisticas
