# D435 approximation for Piper welding simulation

The welding-gun profile uses separate Gazebo RGB and depth sensors attached to
the nominal D435 color/depth frames from `realsense2_description` (15 mm offset;
50 mm stereo baseline). The calibrated wrist mount itself is preserved.

| Property | Profile |
| --- | --- |
| RGB | 1920 × 1080, 30 Hz, 69.4° × 42.5° nominal FOV |
| Native depth | 1280 × 720, 30 Hz, 87° × 58° nominal FOV |
| Depth output | 16UC1; 1 mm units; zero means invalid |
| Depth range | 0.28–10 m (configurable processing gate, not guaranteed useful accuracy) |
| Point cloud | Native depth grid transformed into `camera_color_optical_frame`, actual RGB image projection, up to 10 Hz |

The sensor rate is in simulation time; wall-clock throughput depends on GPU/CPU
load and the Gazebo real-time factor. RGB and depth CameraInfo are generated
from the intrinsics used to render each sensor. ROS bridge reliability is kept
compatible with existing GUI/WebRTC and detector subscriptions.

## Topics

- `/piper/camera/image_raw`, `/piper/camera/camera_info`: native RGB and intrinsics.
- `/piper/camera/depth/ideal_image`: geometric floating-point depth in metres.
- `/piper/camera/depth/camera_info`: native depth intrinsics.
- `/piper/camera/depth/image_raw`: modeled depth in integer millimetres.
- `/piper/camera/points_reframed` and legacy `/piper/camera/points`: colored points
  in color optical coordinates. Points outside RGB FOV retain XYZ and have black
  color; registration does not just relabel the native depth frame.

## Approximation and tuning

`d435_depth_processing.py` adds Gaussian disparity noise with adjustable sigma
(default 0.08 px), converts back to depth using the 50 mm baseline, quantizes to
1 mm and rejects out-of-range pixels. This yields distance-dependent uncertainty
rather than constant metric noise. RGB uses a small Gaussian image noise
(default 0.002 normalized intensity, xacro arg `d435_rgb_noise_stddev`). These
noise settings are illustrative, not measurements of the user's camera.

No physical D435 was accessible when this profile was made. The intrinsics are
nominal rectified pinhole values, not the unit's factory calibration. Gazebo does
not reproduce D435 active stereo matching, the IR speckle projector, exposure/
gain response, RGB rolling shutter, motion blur, material-dependent depth holes,
stereo edge occlusion failures or calibrated lens distortion. Consequently this
profile tests geometry, registration and software flow but cannot certify
real-camera precision. A measured camera profile and flat-target datasets would
be needed to fit those effects. RealSense recommends 848 × 480 for optimal D435
depth operation; this profile selects the supported 1280 × 720 HD output mode.

Sources:
- [Intel D435 product brief](https://simplecore.intel.com/realsensehub/wp-content/uploads/sites/63/D435_Series_ProductBrief_010718.pdf)
- [RealSense depth tuning and disparity noise](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/)
- [Gazebo depth sensor implementation](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/DepthCameraSensor.cc)

Tests cover invalid geometry, metric units, quantization, quadratic noise growth,
RGB projection after an extrinsic translation and the expanded robot model's
separate sensor frames, resolutions and FOVs.
