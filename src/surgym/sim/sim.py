from hydra.utils import to_absolute_path
from isaacgym import gymapi
from omegaconf import DictConfig

from surgym.sim.scenes import scenes


class Simulator:
    def __init__(self, cfg: DictConfig, sim_type):
        """Simulation

        Args:
            cfg (DictConfig): Configuration
            sim_type (gymapi.SIM_FLEX/gymapi.SIM_PHYSX): Simulation type
        """

        self.cfg = cfg
        self.sim_type = sim_type

        # Create simulation from scene
        self.gym = gymapi.acquire_gym()
        self.sim = self._create()
        self.scene = scenes[cfg.run.robot](cfg, self.gym, self.sim)
        self.gym.prepare_sim(self.sim)

        # Get environment and camera/viewer handles
        self.envs = self.scene.envs
        if self.cfg.run.save_img:
            self.cam = self.scene.cam
        if not self.cfg.run.headless:
            self.viewer = self.scene.viewer

    def _create(self):
        if self.cfg.run.headless and not self.cfg.run.save_img:
            graphics_device = -1  # Disable rendering
        else:
            graphics_device = self.cfg.run.graphics_device

        # Set simulation parameters
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / self.cfg.run.fps
        sim_params.substeps = self.cfg.run.substeps
        sim_params.stress_visualization = self.cfg.run.viz_stress
        sim_params.stress_visualization_min = self.cfg.run.viz_stress_min
        sim_params.stress_visualization_max = self.cfg.run.viz_stress_max
        sim_params.up_axis = gymapi.UP_AXIS_Y
        sim_params.use_gpu_pipeline = self.cfg.run.use_gpu_pipeline
        sim_params.flex.solver_type = self.cfg.flex.solver_type
        sim_params.flex.num_outer_iterations = self.cfg.flex.num_outer_iterations
        sim_params.flex.num_inner_iterations = self.cfg.flex.num_inner_iterations
        sim_params.flex.relaxation = self.cfg.flex.relaxation
        sim_params.flex.shape_collision_margin = self.cfg.flex.shape_collision_margin

        # Create simulator
        sim = self.gym.create_sim(
            self.cfg.run.compute_device,
            graphics_device,
            type=self.sim_type,
            params=sim_params,
        )
        assert sim is not None, "Simulation failed to initialize"

        return sim

    def run(self):
        while not self.gym.query_viewer_has_closed(self.viewer):
            # while True:

            # Step simulation
            self.gym.simulate(self.sim)
            self.gym.fetch_results(self.sim, True)
            sim_time = self.gym.get_sim_time(self.sim)

            # Render camera sensor
            self.gym.render_all_camera_sensors(self.sim)
            self.gym.step_graphics(self.sim)

            # Update Viewer (OPTIONAL)
            if not self.cfg.run.headless:
                self.gym.draw_viewer(self.viewer, self.sim, False)

                # Wait for dt to elapse in real time to sync viewer with simulation rate
                self.gym.sync_frame_time(self.sim)

            # Save image to disk
            if self.cfg.run.save_img:
                frame_count = self.gym.get_frame_count(self.sim)
                if round(frame_count % (round(self.cfg.run.fps) / self.cfg.run.save_fps)) == 0:
                    # elasped_time = frame_count // round(sim_fps)
                    img_path = to_absolute_path(f"images/img_{frame_count:05d}.png")
                    self.gym.write_camera_image_to_file(
                        self.sim,
                        self.envs,
                        self.cam,
                        gymapi.IMAGE_COLOR,  # IMAGE_COLOR/IMAGE_DEPTH/IMAGE_SEGMENTATION/IMAGE_OPTICAL_FLOW
                        img_path,
                    )

            # Log stress
            if self.cfg.run.compute_stress:
                tet_indices, tet_stress = self.gym.get_sim_tetrahedra(self.sim)
                tri_indices, tri_parents, tri_normals = self.gym.get_sim_triangles(self.sim)

                # TODO: Visualize stress

            if sim_time >= self.cfg.run.timeout:
                break

        if not self.cfg.run.headless:
            self.gym.destroy_viewer(self.viewer)

        self.gym.destroy_sim(self.sim)
