
# Mobile robot primitive mapping using computer vision

This software is designed to facilitate the detection and mapping of primitives in a simulated environment using Webots. The project includes tools for both manual and automatic detection processes, allowing users to generate detailed maps and analyze them through various provided scripts. Developed with compatibility for Windows and Linux systems, this project leverages Python and several of its libraries to process and visualize data efficiently.

## Key Features

- **Map Generation:** Automated creation of maps with customizable parameters.
- **Manual Detection:** Interactive detection and analysis of primitives using keyboard controls within Webots.
- **Automatic Detection:** Fully automated detection that streamlines the process and outputs detailed accuracy metrics.

This README provides comprehensive guidance on setting up the project, installing dependencies, and running the various components to maximize functionality and ease of use.


## Compatibility

This project is developed on **Windows 11 Home 22H2** but is compatible with the following operating systems:
- **Windows 10**
- **Linux 20.04.6 LTS (Focal Fossa)**
- **Python Version: 3.10**

It requires **Webots version R2023B** to run as intended.

## Dependencies

The project is implemented in Python. The following Python modules are necessary to run the project:
- `numpy==1.26.4`
- `scikit-learn==1.4.2`
- `matplotlib==3.8.4`
- `pyransac3d==1.1.3`
- `scikit-image==0.22.0`


Make sure to install these dependencies using **pip install -r requirements.txt**

## Running the Code

### Generating Maps

- **Script:** `mapa.py`
- **Functionality:** Generates maps and saves them along with annotations in JSON format.
- **Usage:** Run the `main` function in the script with parameters specifying the number of maps (`num_maps`) and the number of figures to include in each map (`num_quadrants`). Maps and their annotations are saved to `updated_map_path` and `annotations/worlds` respectively.

### Manual Primitive Detection

- **Script:** `gps_lidar.py`
- **Setup:** 
  1. Open the Webots environment and load the desired map.
  2. Modify `constants.py` to set the results file name.
  3. Run the script in PyCharm or your preferred IDE.
- **Operation:** 
  1. Enter the map name and the number of figures in the terminal (the number is used only to organize the results file name).
  2. Control the movement in Webots using `W`, `A`, `S`, `D`.
  3. Press `P` to plot current scan results or `L` to finalize the scan and save results including total accuracy, figure accuracy, position accuracy, measurement accuracy and time to the file specified in `constants.py`.

### Automatic Primitive Detection

- **Script:** `automatic_solver.py`
- **Setup:** 
  1. Open the Webots environment and load the desired map.
  2. Modify `constants.py` to set the results file name.
  3. Run the script in PyCharm or your preferred IDE.
- **Operation:** 
  1. Enter the map name and the number of figures in the terminal (the number is used only to organize the results file name).
  2. The script automatically performs scans and saves results including total accuracy, figure accuracy, position accuracy, measurement accuracy and time to the file specified in `constants.py`.

## Additional Information

Ensure that all file paths and environment settings are correctly configured according to your system setup and file structure to avoid runtime errors.




