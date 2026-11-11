# RSSI monitor + exploration coordinator image:
# mesh_sim:humble (ROS 2 Humble + numpy/scipy) + scikit-fmm + scikit-image (road graph).
# Derived rather than modifying mesh_sim:humble, which also runs on NUC2.
# numpy stays at the base image's 1.21.5: the system matplotlib/cv_bridge break on numpy 2.
#   docker build -t fair_rssi:humble rssi/ros
FROM mesh_sim:humble
RUN pip3 install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir "scikit-fmm" "scikit-image==0.21.0" "numpy==1.21.5" \
 && python3 -c "import skfmm, skimage, numpy, scipy; print('skfmm', skfmm.__version__, 'skimage', skimage.__version__, 'numpy', numpy.__version__)"
