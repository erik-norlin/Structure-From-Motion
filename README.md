# Structure-From-Motion

This repository contains a computer vision project implementing Structure from Motion, a technique for reconstructing 3D scenes from 2D images. The implementation involves several steps including rotation averaging, translation registration, camera refinement, and final 3D reconstruction.

## Algorithm

### Rotation Averaging
The algorithm estimates absolute rotations of cameras by chaining relative rotations of adjacent camera pairs. The relative rotations are obtained from robustly estimated essential matrices using image correspondences obtained from SIFT in a RANSAC loop.

### Translation Registration
An initial 3D reconstruction is triangulated with a camera pair with a sufficiently large base line and image correspondences for these cameras. Camera translations are robustly estimated in a RANSAC loop and obtained using 2D-3D correspondeces and absolute rotations.

### Camera Refinement
The absolute rotations and translations are refined by minimizing the squared reprojection error using Levenberg-Marquardt (LM).

### Final 3D Reconstruction
Two final 3D reconstructions are obtained by accumulating triangulated 3D points using the refined cameras and image correspondences for each adjacent camera pair. The first shows the reconstruction of the unrefined camera pairs, and the second shows the reconstruction of the refined camera pairs.

## Running the Software

To run the software, make sure the following modules are installed (also contained in `requirements.txt`):
- argparse
- matplotlib
- numpy
- opencv-python
- scipy

The main files are:
- `computer_vision.py`
- `get_dataset_info.py`
- `main.py`
- `pipeline.py`

Run the software with `main.py -dataset=<dataset>`. Use the `-dataset` flag to specify the dataset (an integer).

## Reconstruction Results

Reconstructions for each dataset before and after LM optimization are available in the repository. The reconstructions visually improve after optimization, aligning point clouds more accurately. The following is an example of how this software can reconstruct structure from motion.

This sequence of images (dataset 3)
![](https://github.com/erik-norlin/Structure-From-Motion/blob/main/reconstruction-plots/dataset_3_joined.png?raw=true)
yields
![](https://github.com/erik-norlin/Structure-From-Motion/blob/main/reconstruction-plots/dataset_2_after_LM_2.png?raw=true)

Further details about the implementation can be found in the code files.
