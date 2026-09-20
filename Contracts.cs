namespace StackPlanner.Core;

/// <summary>
/// 箱子在供包、运输和堆垛流程中的业务状态。
/// </summary>
public enum BoxStatus
{
    /// <summary>箱子仍在货架上，可以参与顺序重新规划。</summary>
    OnShelf,
    /// <summary>箱子已经从货架取下，但尚未到达堆垛位置。</summary>
    RemovedFromShelf,
    /// <summary>箱子已经到达堆垛工位，等待机械执行。</summary>
    Arrived,
    /// <summary>机械正在执行该箱子的堆垛动作。</summary>
    Stacking,
    /// <summary>箱子已经实际放置成功，形成托盘上的真实固定占用。</summary>
    StackingSucceeded,
    /// <summary>箱子的本次堆垛动作失败，箱子仍保留在集合中等待业务处理。</summary>
    StackingFailed,
}

/// <summary>
/// 独立箱子的输入属性和由 DLL 维护的业务规划状态。
/// 每个实例代表一个真实箱子，不与其他箱子合并数量。
/// </summary>
public sealed record Box
{
    /// <summary>箱子的唯一业务编号；同一个 <see cref="BoxGroup" /> 内不能重复。</summary>
    public required string BoxNumber { get; init; }
    /// <summary>箱子重量，单位为 kg；当前版本用于输入校验，重量约束后续接入。</summary>
    public double WeightKg { get; init; }
    /// <summary>箱子长度，单位为 mm。</summary>
    public double LengthMm { get; init; }
    /// <summary>箱子宽度，单位为 mm。</summary>
    public double WidthMm { get; init; }
    /// <summary>箱子高度，单位为 mm。</summary>
    public double HeightMm { get; init; }
    /// <summary>当前业务状态；新箱子默认为 <see cref="BoxStatus.OnShelf" />。</summary>
    public BoxStatus Status { get; set; } = BoxStatus.OnShelf;
    /// <summary>
    /// 堆垛顺序，从 0 开始；-1 表示尚未分配。
    /// 非 <see cref="BoxStatus.OnShelf" /> 箱子的顺序视为已锁定。
    /// </summary>
    public int Order { get; set; } = -1;
}

/// <summary>
/// 当前任务中的独立箱子集合。
/// 集合本身不按照箱型合并，也不保存规划器之外的设备状态。
/// </summary>
public sealed class BoxGroup
{
    private readonly List<Box> _boxes = new();
    /// <summary>以只读视图提供当前箱子，防止调用方直接替换集合。</summary>
    public IReadOnlyList<Box> Boxes => _boxes;
    /// <summary>供 <see cref="StackPlanner" /> 内部执行增删改的可变集合。</summary>
    internal List<Box> MutableBoxes => _boxes;
}

/// <summary>
/// 单个箱子的堆垛规划结果。
/// 坐标为毫米，位置使用箱体中心点，朝向只允许 0° 或 90°。
/// </summary>
public sealed record BoxPlacement
{
    /// <summary>该箱子的堆垛顺序。</summary>
    public required int Order { get; init; }
    /// <summary>该位置对应的唯一箱号。</summary>
    public required string BoxNumber { get; init; }
    /// <summary>箱体中心在托盘坐标系中的 X 坐标，单位为 mm。</summary>
    public required double Xmm { get; init; }
    /// <summary>箱体中心在托盘坐标系中的 Y 坐标，单位为 mm。</summary>
    public required double Ymm { get; init; }
    /// <summary>机械执行使用的 Z 坐标，单位为 mm。</summary>
    public required double Zmm { get; init; }
    /// <summary>箱子的平面朝向，只能是 0 或 90。</summary>
    public required int OrientationDeg { get; init; }
    /// <summary>从托盘底面开始计算的层号，底层为 0。</summary>
    public required int LayerIndex { get; init; }
}

/// <summary>
/// 一次完整规划的公开结果。
/// 结果只包含应用层执行所需的放置项和整体成功标志。
/// </summary>
public sealed class StackPlanResult
{
    /// <summary>按规划顺序排列的箱子放置结果。</summary>
    public required IReadOnlyList<BoxPlacement> Placements { get; init; }
    /// <summary>所有箱子都成功获得合法位置时为 true，否则为 false。</summary>
    public required bool PlanningResult { get; init; }
}

/// <summary>
/// 指定尺寸箱子的剩余容量查询结果。
/// </summary>
public sealed class RemainingCapacityResult
{
    /// <summary>查询箱子的长度，单位为 mm。</summary>
    public required double LengthMm { get; init; }
    /// <summary>查询箱子的宽度，单位为 mm。</summary>
    public required double WidthMm { get; init; }
    /// <summary>查询箱子的高度，单位为 mm。</summary>
    public required double HeightMm { get; init; }
    /// <summary>当前箱子全部具备合法规划位置时为 true。</summary>
    public required bool CurrentPlanValid { get; init; }
    /// <summary>当前状态下最多还能完整放置的同尺寸箱子数量。</summary>
    public required int MaxAdditionalCount { get; init; }
    /// <summary>查询是否完成；输入尺寸非法或当前基础规划不可用时为 false。</summary>
    public required bool QuerySucceeded { get; init; }
    /// <summary>查询结果说明。</summary>
    public required string Message { get; init; }
}
