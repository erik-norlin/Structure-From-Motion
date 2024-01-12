# Structure-From-Motion

This repository contains a computer vision project implementing Structure from Motion, a technique for reconstructing 3D scenes from 2D images. The implementation involves several steps including rotation averaging, translation registration, camera refinement, and final 3D reconstruction.

## Algorithm

### Rotation Averaging
The algorithm estimates absolute rotations of cameras by chaining relative rotations of adjacent camera pairs. The relative rotations are obtained from robustly estimated essential matrices using image correspondences obtained from SIFT points in a RANSAC loop.

### Translation Registration
An initial 3D reconstruction is triangulated with a camera pair with a sufficiently large base line and SIFT matches. Camera translations are robustly estimated in a RANSAC loop and obtained by solving DLT using 2D-3D corespondeces and absolute rotations.

### Camera Refinement
The absolute rotations and translations are refined by minimizing the squared reprojection error using Levenberg-Marquardt (LM).

### Final 3D Reconstruction
Two final 3D reconstructions are obtained by accumulating triangulated 3D points using the refined cameras and image correspondences for each adjacent camera pair. The first shows the reconstruction of the unrefined camera pairs, and the second shows the reconstruction of the refined camera pairs.

## Running the Software

To run the software, make sure the following modules are installed:
- argparse
- matplotlib
- numpy
- opencv-python
- scipy
- tqdm

The main files are:
- `computer_vision.py`
- `get_dataset_info.py`
- `main.py`
- `pipeline.py`

Run the software with `main.py -dataset <dataset> -T_robust`. Use the `-dataset` flag to specify the dataset (an integer), and `-T_robust` as an optional flag for robust translation estimation.

## Reconstruction Results

Reconstructions for each dataset before and after LM optimization are available in the repository. The reconstructions visually improve after optimization, aligning point clouds more accurately.

### With Robust Translations
- Figures: `dataset_2_before_LM_2_robust_T.png` to `dataset_9_after_LM_2_robust_T.png`

### Without Robust Translations

![Reconstruction Example]](https://github.com/erik-norlin/Structure-From-Motion/blob/main/reconstruction-plots/dataset_2_after_LM_2.png?raw=true))
![](https://github.com/erik-norlin/TME290-Autonomous-Robots/blob/main/assignment3/Assignment3_Erik_Norlin/python_kinematic/plot/plots/path_taken.png?raw=true)


For each dataset, the reconstructions with non-robust translations generally show better results after LM optimization compared to reconstructions with robust translations.

## Conclusion

While the implementation works well for most datasets, challenges were encountered, especially with the robust translation estimation and LM optimization. Compromises were made to achieve satisfactory reconstructions. Further details about the implementation can be found in the code files.
