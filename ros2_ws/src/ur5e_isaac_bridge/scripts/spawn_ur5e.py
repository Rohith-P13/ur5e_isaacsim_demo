# Spawn the official UR description into Isaac Sim and publish joint state
# over the ROS 2 bridge. Run with Isaac Sim's Python, for example:
#   ~/isaacsim/python.sh spawn_ur5e.py --xacro /path/ur_isaac.urdf.xacro

import argparse
import os
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--xacro", required=True, help="Path to ur_isaac.urdf.xacro")
parser.add_argument("--ur-type", default="ur5e")
parser.add_argument("--joint-commands-topic", default="/isaac_joint_commands")
parser.add_argument("--joint-states-topic", default="/isaac_joint_states")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--camera-namespace", default="/wrist_cam")
parser.add_argument("--camera-frame", default="camera_optical_frame")
parser.add_argument("--camera-width", type=int, default=640)
parser.add_argument("--camera-height", type=int, default=480)
parser.add_argument("--camera-hfov-deg", type=float, default=60.0)
parser.add_argument("--camera-rate", type=float, default=15.0, help="Publish rate in Hz")
parser.add_argument(
    "--cube-xyz",
    type=float,
    nargs=3,
    default=[0.5, 0.0, 0.2],
    help="Target cube centre in world (= base_link) coordinates, metres",
)
parser.add_argument("--cube-size", type=float, default=0.05, help="Cube edge, metres")
parser.add_argument("--no-cube", action="store_true", help="Do not add the target cube")
parser.add_argument(
    "--initial-positions",
    default=None,
    help="YAML of joint name -> radians to spawn the arm in. Defaults to "
    "ur_description/config/initial_positions.yaml, the same file ros2_control uses.",
)
args, _unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"renderer": "RealTimePathTracing", "headless": args.headless}
)

import math
import yaml

import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import RtxCamera
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def expand_urdf() -> str:
    """Expand the official xacro with absolute mesh paths for the importer."""
    out_dir = tempfile.mkdtemp(prefix="ur5e_isaac_urdf_")
    urdf_path = os.path.join(out_dir, "ur5e.urdf")
    command = [
        "xacro",
        args.xacro,
        f"ur_type:={args.ur_type}",
        "name:=ur",
        "force_abs_paths:=true",
        f"joint_commands_topic:={args.joint_commands_topic}",
        f"joint_states_topic:={args.joint_states_topic}",
    ]
    result = subprocess.run(command, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        raise RuntimeError(
            "xacro failed. Source the ROS 2 workspace before launching Isaac Sim."
        )
    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(result.stdout)
    return urdf_path


def articulation_path(hint: str) -> str:
    stage = stage_utils.get_current_stage(backend="usd")
    prim = stage.GetPrimAtPath(hint)
    if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return hint
    if prim and prim.IsValid():
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(child.GetPath())
    return hint


def add_wrist_camera(robot_root: str) -> str:
    """Create the wrist camera under the imported camera optical frame link.

    The URDF owns the camera pose (tool0 -> camera_link -> camera_optical_frame),
    so the USD camera is parented to that link with no offset of its own. The
    only local transform is the fixed axis flip between the two conventions:
    a ROS optical frame looks along +Z with +Y down, a USD camera looks along
    -Z with +Y up. That is a 180 degree rotation about X.
    """
    stage = stage_utils.get_current_stage(backend="usd")
    optical_prim = next(
        (
            prim
            for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root))
            if prim.GetName() == args.camera_frame
        ),
        None,
    )
    if optical_prim is None:
        raise RuntimeError(
            f"Could not find the {args.camera_frame} link in the imported robot."
        )

    camera_path = f"{optical_prim.GetPath()}/WristCamera"
    # tick_rate sets omni:sensor:tickRate, which is how the ROS 2 camera
    # helpers are throttled on this Isaac Sim version.
    RtxCamera(camera_path, tick_rate=args.camera_rate)

    camera_prim = stage.GetPrimAtPath(camera_path)
    xformable = UsdGeom.Xformable(camera_prim)
    xformable.ClearXformOpOrder()
    xformable.AddRotateXOp().Set(180.0)

    # Pinhole intrinsics. Square pixels: the aperture ratio matches the image
    # aspect ratio, so fx == fy == width / (2 * tan(hfov / 2)).
    camera = UsdGeom.Camera(camera_prim)
    horizontal_aperture = 20.955  # USD default, in tenths of a stage unit
    focal_length = horizontal_aperture / (
        2.0 * math.tan(math.radians(args.camera_hfov_deg) / 2.0)
    )
    camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
    camera.GetVerticalApertureAttr().Set(
        horizontal_aperture * args.camera_height / args.camera_width
    )
    camera.GetFocalLengthAttr().Set(focal_length)
    # The USD default near plane is 1 stage unit (1 m here), which would clip
    # everything a wrist camera is meant to look at.
    camera.GetClippingRangeAttr().Set((0.01, 100.0))

    print(f"Wrist camera created: {camera_path}")
    return camera_path


def find_initial_positions_file() -> str | None:
    if args.initial_positions:
        return args.initial_positions
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        candidate = os.path.join(prefix, "share", "ur_description", "config", "initial_positions.yaml")
        if os.path.isfile(candidate):
            return candidate
    return None


def set_initial_joint_positions(robot_root: str) -> None:
    """Start the simulation with the arm already in its ros2_control start pose.

    The importer leaves every joint at 0, which is the arm stretched out
    horizontally with the wrist at table height. ros2_control only lifts it
    to the start pose after Play, and with the base correctly at z = 0 the
    wrist starts inside the Simple Room table and gets pushed underneath it.
    Setting the joint state and the drive target before Play avoids that.
    """
    path = find_initial_positions_file()
    if path is None:
        print("No initial_positions.yaml found; joints start at 0.")
        return
    with open(path, encoding="utf-8") as f:
        positions = yaml.safe_load(f)
    stage = stage_utils.get_current_stage(backend="usd")
    applied = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        name = prim.GetName()
        if name not in positions or not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        degrees = math.degrees(float(positions[name]))  # USD angular units are degrees
        prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(degrees)
        prim.CreateAttribute("state:angular:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)
        UsdPhysics.DriveAPI.Apply(prim, "angular").CreateTargetPositionAttr().Set(degrees)
        applied.append(name)
    print(f"Initial joint positions from {path}: {sorted(applied)}")


def bolt_base_to_environment(base_path: str, environment_root: str) -> None:
    """Stop the robot base colliding with the room it is mounted in.

    The base is held by a plain fixed joint, not by the articulation, so
    contact can move it. The UR base collision mesh overlaps the Simple Room
    table, and without this filter PhysX pushes the whole robot up by about
    1.5 cm. TF (from the URDF) does not see that, so every camera pose was off
    by the same 1.5 cm in Z. A real arm is bolted to its table, so filtering
    base-vs-room contacts is the physically right model. Every other link
    still collides with the room.
    """
    stage = stage_utils.get_current_stage(backend="usd")
    root = stage.GetPrimAtPath(environment_root)
    if not root or not root.IsValid():
        return
    colliders = [
        prim.GetPath()
        for prim in Usd.PrimRange(root)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    base = stage.GetPrimAtPath(base_path)
    UsdPhysics.FilteredPairsAPI.Apply(base).CreateFilteredPairsRel().SetTargets(colliders)
    print(f"Filtered {len(colliders)} room colliders against {base_path}")


def add_target_cube(path: str = "/World/TargetCube") -> None:
    """Add a visual-only cube for the camera to look at.

    No collision or rigid body API is applied, so the arm cannot knock it and
    physics ignores it. The capture script adds the same box to the MoveIt
    planning scene so plans still keep clear of it.
    """
    stage = stage_utils.get_current_stage(backend="usd")
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(args.cube_size)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*args.cube_xyz))

    # Bright red so it stands out from the grey room and can be segmented.
    material = UsdShade.Material.Define(stage, path + "/Looks/Red")
    shader = UsdShade.Shader.Define(stage, path + "/Looks/Red/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.9, 0.05, 0.05))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.05, 0.05)])
    print(f"Target cube: {path} at {tuple(args.cube_xyz)}, edge {args.cube_size} m")


def main() -> None:
    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.asset.importer.urdf")
    app_utils.enable_extension("isaacsim.sensors.experimental.rtx")
    simulation_app.update()

    stage_utils.set_stage_units(meters_per_unit=1.0)
    ViewportManager.set_camera_view(
        "/OmniverseKit_Persp",
        eye=np.array([1.6, 1.6, 1.2]),
        target=np.array([0.0, 0.0, 0.4]),
    )

    assets_root = get_assets_root_path()
    if assets_root:
        stage_utils.add_reference_to_stage(
            assets_root + "/Isaac/Environments/Simple_Room/simple_room.usd",
            "/background",
        )

    if not args.no_cube:
        add_target_cube()

    urdf_path = expand_urdf()
    usd_dir = tempfile.mkdtemp(prefix="ur5e_isaac_usd_")
    importer = URDFImporter(
        URDFImporterConfig(
            urdf_path=urdf_path,
            usd_path=usd_dir,
            fix_base=True,
            merge_fixed_joints=False,
            collision_from_visuals=False,
            joint_drive_type="force",
            joint_target_type="position",
            override_joint_stiffness=100000.0,
            override_joint_damping=10000.0,
            run_asset_transformer=False,
        )
    )
    usd_path = importer.import_urdf()
    robot_root = "/ur5e"
    stage_utils.add_reference_to_stage(usd_path, robot_root)
    simulation_app.update()
    robot_path = articulation_path(robot_root)

    camera_path = add_wrist_camera(robot_root)
    bolt_base_to_environment(robot_path, "/background")
    set_initial_joint_positions(robot_root)
    print(f"UR5e articulation: {robot_path}")

    commands_topic = args.joint_commands_topic.lstrip("/")
    states_topic = args.joint_states_topic.lstrip("/")
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("CreateWristCameraRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishWristImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishWristDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishWristCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateWristCameraRenderProduct.inputs:execIn"),
                ("CreateWristCameraRenderProduct.outputs:execOut", "PublishWristImage.inputs:execIn"),
                ("CreateWristCameraRenderProduct.outputs:execOut", "PublishWristDepth.inputs:execIn"),
                ("CreateWristCameraRenderProduct.outputs:execOut", "PublishWristCameraInfo.inputs:execIn"),
                ("CreateWristCameraRenderProduct.outputs:renderProductPath", "PublishWristImage.inputs:renderProductPath"),
                ("CreateWristCameraRenderProduct.outputs:renderProductPath", "PublishWristDepth.inputs:renderProductPath"),
                ("CreateWristCameraRenderProduct.outputs:renderProductPath", "PublishWristCameraInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishWristImage.inputs:context"),
                ("Context.outputs:context", "PublishWristDepth.inputs:context"),
                ("Context.outputs:context", "PublishWristCameraInfo.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", robot_path),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(robot_path)]),
                ("PublishJointState.inputs:topicName", states_topic),
                ("SubscribeJointState.inputs:topicName", commands_topic),
                ("CreateWristCameraRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("CreateWristCameraRenderProduct.inputs:width", args.camera_width),
                ("CreateWristCameraRenderProduct.inputs:height", args.camera_height),
                ("PublishWristImage.inputs:nodeNamespace", args.camera_namespace),
                ("PublishWristImage.inputs:topicName", "image_raw"),
                ("PublishWristImage.inputs:frameId", args.camera_frame),
                ("PublishWristImage.inputs:type", "rgb"),
                ("PublishWristDepth.inputs:nodeNamespace", args.camera_namespace),
                ("PublishWristDepth.inputs:topicName", "depth/image_raw"),
                ("PublishWristDepth.inputs:frameId", args.camera_frame),
                ("PublishWristDepth.inputs:type", "depth"),
                ("PublishWristCameraInfo.inputs:nodeNamespace", args.camera_namespace),
                ("PublishWristCameraInfo.inputs:topicName", "camera_info"),
                ("PublishWristCameraInfo.inputs:frameId", args.camera_frame),
            ],
        },
    )
    # Optional on this Isaac Sim build. The joint graph above matches the
    # shipped ROS 2 bridge sample and must still come up if these inputs differ.
    for attr_path, value in (
        ("/ActionGraph/Context.inputs:useDomainIDEnvVar", True),
        ("/ActionGraph/PublishClock.inputs:topicName", "clock"),
    ):
        try:
            og.Controller.set(og.Controller.attribute(attr_path), value)
        except Exception as exc:
            print(f"Left {attr_path} at its default: {exc}")

    simulation_app.update()
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
