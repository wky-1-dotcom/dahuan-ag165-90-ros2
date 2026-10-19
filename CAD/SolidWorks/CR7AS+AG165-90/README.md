# CR7AS + AG165-90 原始装配

本目录含 47 个原始 SolidWorks 文件和一个 `table.STL` 辅助网格。

- `机械臂+末端执行器.SLDASM`：整机与末端执行器的主装配入口。
- `DOBOT CR7Model*.SLDASM`、`J1-CR7_ASM.SLDASM` 至 `J6-CR7_ASM.SLDASM`：CR7AS 机械臂及各关节装配。
- `ASM*.SLDASM`：末端机构的子装配。
- `本体.SLDPRT`、`left-*.SLDPRT`、`link.SLDPRT`、`NONE-*.SLDPRT`、`PRT*.SLDPRT`：装配所依赖的零件。
- `table.STL`：辅助场景网格，不是夹爪零件。

文件名来自原始交付，不能仅凭名称推断零件功能或尺寸。修改前请在 SolidWorks 中打开顶层装配并检查引用是否完整；重新导出网格后，还应同步检查 URDF/SDF 的单位、原点、质量和碰撞体。
