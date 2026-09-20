namespace StackPlanner.Core;

/// <summary>
/// 纯堆垛集合和确定性规划器。DLL 内部维护集合、顺序、当前方案和成功箱真实占用。
/// </summary>
public sealed class StackPlanner
{
    /// <summary>托盘可用区域的 X 最小值，单位为 mm。</summary>
    public const double PalletXMinMm = 1300;
    /// <summary>托盘可用区域的 X 最大值，单位为 mm。</summary>
    public const double PalletXMaxMm = 2400;
    /// <summary>托盘可用区域的 Y 最小值，单位为 mm。</summary>
    public const double PalletYMinMm = -1000;
    /// <summary>托盘可用区域的 Y 最大值，单位为 mm。</summary>
    public const double PalletYMaxMm = 100;
    /// <summary>相邻箱子之间的规划间隙，单位为 mm。</summary>
    public const double StackBoxGapMm = 20;
    /// <summary>托盘允许的最大堆垛高度，单位为 mm。</summary>
    public const double StackMaxHeightMm = 1500;
    /// <summary>托盘底面对应的机械 Z 基准值，单位为 mm。</summary>
    public const double PlaceFloorZMm = 1800;
    /// <summary>托盘边缘预留距离，单位为 mm。</summary>
    public const double EdgeMarginMm = 0;

    /// <summary>几何比较使用的固定误差，避免浮点边界导致结果不稳定。</summary>
    private const double Epsilon = 0.0001;
    /// <summary>DLL 当前维护的全部箱子。</summary>
    private readonly BoxGroup _group = new();
    /// <summary>已实际堆垛成功箱子的固定位置，不允许普通重规划改变。</summary>
    private readonly Dictionary<string, FixedPlacement> _succeeded = new(StringComparer.Ordinal);
    /// <summary>最近一次完整规划，用于恢复非 OnShelf 箱子的规划占用。</summary>
    private readonly Dictionary<string, BoxPlacement> _lastPlacements = new(StringComparer.Ordinal);
    /// <summary>对外返回的最近一次规划结果。</summary>
    private StackPlanResult _lastPlan = new() { Placements = Array.Empty<BoxPlacement>(), PlanningResult = true };

    /// <summary>
    /// 获取当前箱子集合的副本。调用方修改返回对象不会影响 DLL 内部状态。
    /// </summary>
    public IReadOnlyList<Box> GetBoxes() => _group.Boxes.Select(CloneBox).ToArray();

    /// <summary>获取最近一次规划结果的副本。</summary>
    public StackPlanResult CurrentPlan => ClonePlan(_lastPlan);

    /// <summary>
    /// 增加一个新箱子。
    /// 新增箱子必须处于 OnShelf 状态，重复箱号不会覆盖已有箱子。
    /// </summary>
    /// <param name="box">待加入的箱子；规划器会复制该对象。</param>
    /// <exception cref="ArgumentNullException">box 为 null。</exception>
    /// <exception cref="ArgumentException">箱号、尺寸或重量不合法。</exception>
    /// <exception cref="InvalidOperationException">箱号重复或初始状态不是 OnShelf。</exception>
    public void AddBox(Box box)
    {
        ArgumentNullException.ThrowIfNull(box);
        ValidateBox(box);
        if (_group.MutableBoxes.Any(x => string.Equals(x.BoxNumber, box.BoxNumber, StringComparison.Ordinal)))
            throw new InvalidOperationException($"箱号已存在: {box.BoxNumber}");
        if (box.Status != BoxStatus.OnShelf)
            throw new InvalidOperationException("新增箱子必须处于 OnShelf 状态。");
        _group.MutableBoxes.Add(CloneBox(box));
    }

    /// <summary>
    /// 批量增加箱子。方法先完整校验输入，任何一个箱子不合法时都不会加入部分数据。
    /// </summary>
    /// <param name="boxes">待加入的独立箱子集合。</param>
    public void AddBoxes(IEnumerable<Box> boxes)
    {
        ArgumentNullException.ThrowIfNull(boxes);
        var incoming = boxes.Select(CloneBox).ToArray();
        var numbers = new HashSet<string>(StringComparer.Ordinal);
        foreach (var box in incoming)
        {
            ValidateBox(box);
            if (box.Status != BoxStatus.OnShelf)
                throw new InvalidOperationException("新增箱子必须处于 OnShelf 状态。");
            if (!numbers.Add(box.BoxNumber) || _group.MutableBoxes.Any(x => x.BoxNumber == box.BoxNumber))
                throw new InvalidOperationException($"箱号已存在: {box.BoxNumber}");
        }
        _group.MutableBoxes.AddRange(incoming);
    }

    /// <summary>
    /// 删除指定箱子并压缩后续箱子的顺序。
    /// Stacking 和 StackingSucceeded 状态的箱子不能删除。
    /// </summary>
    /// <param name="boxNumber">待删除的唯一箱号。</param>
    /// <returns>成功删除返回 true；箱号不存在或状态受保护时返回 false。</returns>
    public bool RemoveBox(string boxNumber)
    {
        var box = FindBox(boxNumber);
        if (box is null || box.Status is BoxStatus.StackingSucceeded or BoxStatus.Stacking)
            return false;
        int removedOrder = box.Order;
        _group.MutableBoxes.Remove(box);
        foreach (var following in _group.MutableBoxes.Where(x => x.Order > removedOrder))
            following.Order--;
        _succeeded.Remove(boxNumber);
        _lastPlacements.Remove(boxNumber);
        return true;
    }

    /// <summary>
    /// 清空整个堆垛任务，包括成功箱真实占用、最近方案和所有箱子状态。
    /// 这是显式的整垛重置操作，清空后可以在同一个实例中开始新任务。
    /// </summary>
    public void ClearBoxes()
    {
        _group.MutableBoxes.Clear();
        _succeeded.Clear();
        _lastPlacements.Clear();
        _lastPlan = new StackPlanResult { Placements = Array.Empty<BoxPlacement>(), PlanningResult = true };
    }

    /// <summary>
    /// 更新箱子的业务状态，并在状态变为 StackingSucceeded 时固化其当前规划位置。
    /// 该方法只记录业务动作结果，不执行机械动作。
    /// </summary>
    /// <param name="boxNumber">待更新的唯一箱号。</param>
    /// <param name="status">新的业务状态。</param>
    /// <returns>箱号存在并完成更新返回 true，否则返回 false。</returns>
    public bool UpdateBoxStatus(string boxNumber, BoxStatus status)
    {
        var box = FindBox(boxNumber);
        if (box is null)
            return false;
        box.Status = status;
        if (status == BoxStatus.StackingSucceeded)
        {
            if (!_lastPlacements.TryGetValue(boxNumber, out var placement))
                throw new InvalidOperationException($"箱子 {boxNumber} 没有可确认的规划位置。");
            _succeeded[boxNumber] = new FixedPlacement(ClonePlacement(placement), box.LengthMm, box.WidthMm, box.HeightMm);
        }
        else
        {
            _succeeded.Remove(boxNumber);
        }
        return true;
    }

    /// <summary>
    /// 根据当前箱子状态生成完整、确定性的堆垛方案。
    /// </summary>
    /// <remarks>
    /// 规划顺序为：先恢复成功箱真实占用，再恢复其他非 OnShelf 箱子的已有规划占用，
    /// 最后为 OnShelf 箱子寻找新位置。非成功箱的规划占用只用于避让，不能作为真实支撑面。
    /// </remarks>
    /// <returns>包含所有可规划放置项和整体 PlanningResult 的结果。</returns>
    public StackPlanResult GeneratePlan()
    {
        AssignOrders();
        var all = _group.MutableBoxes.OrderBy(x => x.Order).ThenBy(x => x.BoxNumber, StringComparer.Ordinal).ToArray();
        var occupied = new List<PlacedBox>();
        var output = new Dictionary<string, BoxPlacement>(StringComparer.Ordinal);
        bool success = true;

        // 先恢复真实占用。真实成功箱的位置不能被后续规划改变。
        foreach (var box in all.Where(x => x.Status == BoxStatus.StackingSucceeded))
        {
            if (!_succeeded.TryGetValue(box.BoxNumber, out var fixedData))
            {
                success = false;
                continue;
            }
            var placed = ToPlacedBox(box, fixedData.Placement, true);
            if (!Fits(placed, occupied)) success = false;
            occupied.Add(placed);
            output[box.BoxNumber] = fixedData.Placement;
        }

        // 非货架箱子优先恢复已有方案；没有旧位置时按固定 Order 确定性重建规划占用。
        foreach (var box in all.Where(x => x.Status != BoxStatus.OnShelf && x.Status != BoxStatus.StackingSucceeded))
        {
            BoxPlacement? placement = null;
            if (_lastPlacements.TryGetValue(box.BoxNumber, out var old))
                placement = old;
            if (placement is null)
                placement = FindPlacement(box, occupied);
            if (placement is null)
            {
                success = false;
                continue;
            }
            var placed = ToPlacedBox(box, placement, false);
            if (!Fits(placed, occupied)) success = false;
            occupied.Add(placed);
            output[box.BoxNumber] = placement;
        }

        // OnShelf 箱子可以重排，但必须避开所有已经恢复的规划占用。
        foreach (var box in all.Where(x => x.Status == BoxStatus.OnShelf).OrderBy(x => x.Order))
        {
            var placement = FindPlacement(box, occupied);
            if (placement is null)
            {
                success = false;
                continue;
            }
            occupied.Add(ToPlacedBox(box, placement, false));
            output[box.BoxNumber] = placement;
        }

        var placements = output.Values.OrderBy(x => x.Order).ThenBy(x => x.BoxNumber, StringComparer.Ordinal).ToArray();
        foreach (var placement in placements)
        {
            var box = FindBox(placement.BoxNumber)!;
            box.Order = placement.Order;
        }
        _lastPlacements.Clear();
        foreach (var placement in placements) _lastPlacements[placement.BoxNumber] = ClonePlacement(placement);
        _lastPlan = new StackPlanResult { Placements = placements, PlanningResult = success && placements.Length == all.Length };
        return ClonePlan(_lastPlan);
    }

    private void AssignOrders()
    {
        // 非 OnShelf 箱子的 Order 是业务流程已经锁定的顺序，不能重新编号。
        // OnShelf 箱子按照原有顺序和箱号排序后，填入剩余的最小可用顺序号，
        // 从而不依赖调用方传入集合的顺序。
        var boxes = _group.MutableBoxes;
        var used = boxes.Where(x => x.Status != BoxStatus.OnShelf && x.Order >= 0).Select(x => x.Order).ToHashSet();
        int next = 0;
        foreach (var box in boxes.Where(x => x.Status == BoxStatus.OnShelf)
                     .OrderBy(x => x.Order < 0 ? int.MaxValue : x.Order)
                     .ThenBy(x => x.BoxNumber, StringComparer.Ordinal))
        {
            while (used.Contains(next)) next++;
            box.Order = next++;
            used.Add(box.Order);
        }
        if (boxes.Any(x => x.Status != BoxStatus.OnShelf && x.Order < 0))
            throw new InvalidOperationException("非 OnShelf 箱子必须已有固定 Order。");
    }

    private BoxPlacement? FindPlacement(Box box, List<PlacedBox> occupied)
    {
        // 候选点由托盘边界和已占用箱体的边界共同产生，避免连续浮点网格搜索。
        var candidates = new List<Candidate>();
        foreach (var orientation in new[] { 0, 90 })
        {
            // 0° 使用箱子的原始长宽，90° 交换 X/Y 方向尺寸。
            double width = orientation == 0 ? box.WidthMm : box.LengthMm;
            double length = orientation == 0 ? box.LengthMm : box.WidthMm;
            var xValues = new SortedSet<double> { PalletXMinMm + EdgeMarginMm + width / 2 };
            var yValues = new SortedSet<double> { PalletYMinMm + EdgeMarginMm + length / 2 };
            foreach (var item in occupied)
            {
                xValues.Add(item.Left - StackBoxGapMm - width / 2);
                xValues.Add(item.Right + StackBoxGapMm + width / 2);
                yValues.Add(item.Bottom - StackBoxGapMm - length / 2);
                yValues.Add(item.Top + StackBoxGapMm + length / 2);
            }
            foreach (var x in xValues.OrderBy(x => x))
            foreach (var y in yValues.OrderBy(y => y))
            foreach (var baseZ in CandidateHeights(occupied))
            {
                // 候选必须同时满足边界、碰撞和支撑条件。
                var candidate = new Candidate(x, y, baseZ, width, length, box.HeightMm, orientation);
                if (!InsidePallet(candidate) || Collides(candidate, occupied)) continue;
                    var supports = Supporting(candidate, occupied);
                if (baseZ > Epsilon && supports.Count == 0) continue;
                // 托盘底层为第 0 层，叠放层级取支撑箱的最大层级加一。
                int layer = baseZ <= Epsilon ? 0 : supports.Max(x => x.Placement.LayerIndex) + 1;
                candidates.Add(new Candidate(candidate.X, candidate.Y, candidate.BaseZ, candidate.Width,
                    candidate.Length, candidate.Height, orientation, layer, supports.Count));
            }
        }
        // 使用完整且固定的决胜顺序，不能依赖候选首次遍历顺序。
        var best = candidates.OrderBy(x => x.Layer)
            .ThenBy(x => x.BaseZ)
            .ThenByDescending(x => x.SupportLength)
            .ThenBy(x => x.Y)
            .ThenBy(x => x.X)
            .ThenBy(x => x.Orientation)
            .FirstOrDefault();
        if (best is null) return null;
        return new BoxPlacement
        {
            Order = box.Order,
            BoxNumber = box.BoxNumber,
            Xmm = Round(best.X),
            Ymm = Round(best.Y),
            Zmm = Round(PlaceFloorZMm - best.BaseZ - box.HeightMm),
            OrientationDeg = best.Orientation,
            LayerIndex = best.Layer,
        };
    }

    /// <summary>
    /// 返回允许尝试的箱底高度：托盘底面和所有已有箱子的顶部。
    /// 这样候选箱只能落在托盘或同高支撑面上，不会产生悬空位置。
    /// </summary>
    private IEnumerable<double> CandidateHeights(List<PlacedBox> occupied) =>
        new[] { 0d }.Concat(occupied.Select(x => x.BaseZ + x.Height)).Distinct().OrderBy(x => x);

    private static List<PlacedBox> Supporting(Candidate c, List<PlacedBox> occupied)
    {
        // 只有 CanSupport=true 的箱顶才可作为实际支撑面。
        // 非 OnShelf 箱子的规划占用只用于避让，因此不会提供支撑。
        return occupied.Where(x => x.TopZ <= c.BaseZ + Epsilon && Math.Abs(x.TopZ - c.BaseZ) <= Epsilon
                && x.CanSupport
                && OverlapLength(c.X - c.Width / 2, c.X + c.Width / 2, x.Left, x.Right) > Epsilon
                && OverlapLength(c.Y - c.Length / 2, c.Y + c.Length / 2, x.Bottom, x.Top) > Epsilon).ToList();
    }

    /// <summary>检查放置项是否在托盘边界内，并且没有与已有占用发生碰撞。</summary>
    private static bool Fits(PlacedBox item, List<PlacedBox> occupied) => InsidePallet(item) && !Collides(item, occupied);
    private static bool InsidePallet(Candidate x) => x.X - x.Width / 2 >= PalletXMinMm - Epsilon && x.X + x.Width / 2 <= PalletXMaxMm + Epsilon
        && x.Y - x.Length / 2 >= PalletYMinMm - Epsilon && x.Y + x.Length / 2 <= PalletYMaxMm + Epsilon
        && x.BaseZ >= -Epsilon && x.BaseZ + x.Height <= StackMaxHeightMm + Epsilon;
    private static bool InsidePallet(PlacedBox x) => x.Left >= PalletXMinMm - Epsilon && x.Right <= PalletXMaxMm + Epsilon
        && x.Bottom >= PalletYMinMm - Epsilon && x.Top <= PalletYMaxMm + Epsilon && x.BaseZ >= -Epsilon
        && x.TopZ <= StackMaxHeightMm + Epsilon;
    // 候选箱的二维投影按间隙膨胀；只有 Z 方向存在重叠时才算三维碰撞。
    private static bool Collides(Candidate c, List<PlacedBox> occupied) => occupied.Any(x =>
        c.BaseZ < x.TopZ - Epsilon && c.BaseZ + c.Height > x.BaseZ + Epsilon
        && OverlapLength(c.X - c.Width / 2 - StackBoxGapMm / 2, c.X + c.Width / 2 + StackBoxGapMm / 2, x.Left - StackBoxGapMm / 2, x.Right + StackBoxGapMm / 2) > Epsilon
        && OverlapLength(c.Y - c.Length / 2 - StackBoxGapMm / 2, c.Y + c.Length / 2 + StackBoxGapMm / 2, x.Bottom - StackBoxGapMm / 2, x.Top + StackBoxGapMm / 2) > Epsilon);
    // 恢复旧方案时使用同一套碰撞规则，保证规划占用与新候选的判断一致。
    private static bool Collides(PlacedBox c, List<PlacedBox> occupied) => occupied.Any(x =>
        c.BaseZ < x.TopZ - Epsilon && c.TopZ > x.BaseZ + Epsilon
        && OverlapLength(c.Left - StackBoxGapMm / 2, c.Right + StackBoxGapMm / 2, x.Left - StackBoxGapMm / 2, x.Right + StackBoxGapMm / 2) > Epsilon
        && OverlapLength(c.Bottom - StackBoxGapMm / 2, c.Top + StackBoxGapMm / 2, x.Bottom - StackBoxGapMm / 2, x.Top + StackBoxGapMm / 2) > Epsilon);
    private static double OverlapLength(double a1, double a2, double b1, double b2) => Math.Max(0, Math.Min(a2, b2) - Math.Max(a1, b1));
    private static double Round(double value) => Math.Round(value, 3, MidpointRounding.AwayFromZero);

    /// <summary>
    /// 将公开的机械 Z 坐标反算为内部箱底高度 BaseZ，供边界、碰撞和支撑计算使用。
    /// </summary>
    private static PlacedBox ToPlacedBox(Box box, BoxPlacement placement, bool real)
    {
        bool rotated = placement.OrientationDeg == 90;
        double width = rotated ? box.LengthMm : box.WidthMm;
        double length = rotated ? box.WidthMm : box.LengthMm;
        double baseZ = PlaceFloorZMm - placement.Zmm - box.HeightMm;
        return new PlacedBox(placement.Xmm, placement.Ymm, baseZ, width, length, box.HeightMm, placement,
            real, real || box.Status == BoxStatus.OnShelf);
    }

    /// <summary>按唯一箱号查找内部箱子对象。</summary>
    private Box? FindBox(string number) => _group.MutableBoxes.FirstOrDefault(x => x.BoxNumber == number);

    /// <summary>校验箱号、尺寸和重量，阻止无效几何进入规划器。</summary>
    private static void ValidateBox(Box box)
    {
        if (string.IsNullOrWhiteSpace(box.BoxNumber)) throw new ArgumentException("箱号不能为空。", nameof(box));
        if (box.LengthMm <= 0 || box.WidthMm <= 0 || box.HeightMm <= 0) throw new ArgumentException("箱子尺寸必须大于 0。", nameof(box));
        if (box.WeightKg < 0) throw new ArgumentException("箱子重量不能小于 0。", nameof(box));
    }
    private static Box CloneBox(Box x) => new() { BoxNumber = x.BoxNumber, WeightKg = x.WeightKg, LengthMm = x.LengthMm, WidthMm = x.WidthMm, HeightMm = x.HeightMm, Status = x.Status, Order = x.Order };
    private static BoxPlacement ClonePlacement(BoxPlacement x) => x with { };
    private static StackPlanResult ClonePlan(StackPlanResult x) => new() { PlanningResult = x.PlanningResult, Placements = x.Placements.Select(ClonePlacement).ToArray() };

    /// <summary>已堆垛成功箱子的固定位置和尺寸快照。</summary>
    private sealed record FixedPlacement(BoxPlacement Placement, double Length, double Width, double Height);

    /// <summary>
    /// 规划器内部的三维占用。IsReal 表示真实成功占用，CanSupport 表示其顶部能否支撑后续箱子。
    /// </summary>
    private sealed record PlacedBox(double X, double Y, double BaseZ, double Width, double Length, double Height,
        BoxPlacement Placement, bool IsReal, bool CanSupport)
    {
        public double Left => X - Width / 2;
        public double Right => X + Width / 2;
        public double Bottom => Y - Length / 2;
        public double Top => Y + Length / 2;
        public double TopZ => BaseZ + Height;
    }
    /// <summary>待筛选的候选位置及其确定性排序所需的评分信息。</summary>
    private sealed record Candidate(double X, double Y, double BaseZ, double Width, double Length, double Height,
        int Orientation, int Layer = 0, double SupportLength = 0);
}
