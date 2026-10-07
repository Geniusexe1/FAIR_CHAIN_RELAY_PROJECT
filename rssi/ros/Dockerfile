# RSSI monitor node image: mesh_sim:humble (ROS 2 Humble + numpy/scipy) + scikit-fmm.
# Derived rather than modifying mesh_sim:humble, which also runs on NUC2.
#   docker build -t fair_rssi:humble rssi/ros
FROM mesh_sim:humble
RUN pip3 install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir "scikit-fmm" \
 && python3 -c "import skfmm, numpy, scipy; print('skfmm', skfmm.__version__, 'numpy', numpy.__version__)"
