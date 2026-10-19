# AG165-90 Xacro 网格

`base_link.STL` 是夹爪基座；`link1_*`、`link2_*`、`link3_*` 分别为左右两侧四连杆机构的三段活动连杆。`urdf/ag165_macro.xacro` 同时将这些网格用于 visual 和 collision。

若重导 STL，必须检查单位为米、局部原点/朝向不变，并重新验证 SDF pin 的闭环几何。
