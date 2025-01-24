# PC_Registration_Master

PC_Registration_Master is a Python-based application designed to handle and process point cloud data. This tool provides a user-friendly GUI for loading, visualizing, preprocessing, and registering point clouds using traditional and statistical methods. It leverages libraries such as PySide6, Open3D, and Vispy to deliver an interactive experience.

---

## Features

- **Point Cloud Visualization**: 
  - Load and display source and target point clouds interactively.
  - Visualize point clouds in 3D with rotation and zoom capabilities.

- **Point Cloud Preprocessing**:
  - Apply voxel downsampling to reduce point cloud size.
  - Perform statistical outlier removal for cleaner data.

- **Point Cloud Registration**:
  - Coarse registration using PCA (Principal Component Analysis).
  - Fine registration using the Coherent Point Drift (CPD) algorithm.
  - Scale transformation for adjusting the target point cloud size to match the source.

- **Data Export**:
  - Save processed source and target point clouds for later use.
---


## Installation

1. **Clone the Repository**:
   ```bash
    git clone https://github.com/yourusername/HIT-PointCloud-Registration.git
    cd HIT-PointCloud-Registration
   ```
   
2. **Install Dependencies: Ensure Python 3.8 or higher is installed. Run**:
  ```bash
  pip install -r requirements.txt
  ```


## User Interface Controls
- **Load Point Clouds**: 
  - Use ``Select Source Pointcloud`` Path and ``Select Target Pointcloud Path`` to load source and target point clouds.

- **Visualization**:
  - Click ``Show Source`` and ``Show Target`` to visualize point clouds in the embedded 3D canvas.

- **Preprocessing**:
  - Choose "Coarse" or "Fine" registration from the dropdown.
  - Click ``Do`` to preprocess the loaded point clouds.

- **Registration**:
  - Save processed source and target point clouds for later use.
  - Click ``Reg`` to perform registration.
  - View the results using ``Show Coarse Registration`` or ``Show Fine Registration``.
 
- **Export Data**:
  - Save processed point clouds using the ``Save`` buttons.
---

## Example

Use sample point clouds in ``.xyz`` or ``.pcd`` formats to test the tool. The application supports basic formats for point cloud data.

---

## Code Structure
- **HIT_Registration_Master.py**: Main script for running the GUI and processing logic.
- **Ui_MainWindow.py**: GUI layout and design code generated using PySide6.

---

## Author
**Tianze Zhang**

**Email: skyzhang0730@gmail.com**

