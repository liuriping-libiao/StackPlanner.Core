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
    /// <summary>有限宽度搜索每一轮最多保留的布局数量。</summary>
    private const int BeamWidth = 64;
    /// <summary>单个箱子最多保留的候选位置数量。</summary>
    private const int MaxCandidatesPerBox = 32;
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
    /// <param name="actualPlacement">
    /// 箱子实际堆垛成功时的放置结果；为空时使用规划器当前记录的位置。
    /// </param>
    /// <returns>箱号存在并完成更新返回 true，否则返回 false。</returns>
    public bool UpdateBoxStatus(string boxNumber, BoxStatus status, BoxPlacement? actualPlacement = null)
    {
        var box = FindBox(boxNumber);
        if (box is null)
            return false;
        if (actualPlacement is not null)
            SetInternalPlacement(boxNumber, actualPlacement);
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
    /// 在核心内部写入箱子已经实际占用的放置位置。
    /// 外部只通过 <see cref="UpdateBoxStatus(string, BoxStatus, BoxPlacement?)" />
    /// 报告业务状态和成功位置，不直接调用此方法。
    /// </summary>
    /// <param name="boxNumber">箱子的唯一编号。</param>
    /// <param name="placement">箱子已经实际使用的放置位置。</param>
    /// <returns>箱子存在且位置合法时返回 true。</returns>
    private bool SetInternalPlacement(string boxNumber, BoxPlacement placement)
    {
        ArgumentNullException.ThrowIfNull(placement);
        var box = FindBox(boxNumber);
        if (box is null || !string.Equals(boxNumber, placement.BoxNumber, StringComparison.Ordinal))
            return false;
        if (placement.Order < 0 || placement.LayerIndex < 0
            || placement.OrientationDeg is not (0 or 90))
            throw new ArgumentException("箱子放置位置的顺序、层号或朝向不合法。", nameof(placement));

        var placed = ToPlacedBox(box, placement, real: true);
        if (!InsidePallet(placed))
            throw new ArgumentException($"箱子 {boxNumber} 的固定位置超出托盘范围。", nameof(placement));

        box.Order = placement.Order;
        _lastPlacements[boxNumber] = ClonePlacement(placement);
        if (box.Status == BoxStatus.StackingSucceeded)
            _succeeded[boxNumber] = new FixedPlacement(ClonePlacement(placement), box.LengthMm, box.WidthMm, box.HeightMm);
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
        var all = _group.MutableBoxes.OrderBy(x => x.Order).ToArray();
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

        // OnShelf 箱子允许重排：保留多个布局分支，优先大箱子和难放箱子，
        // 在可堆箱数相同的情况下再比较高度和紧凑性。
        // 四个风车箱需要作为整体放置；先规划其他箱子铺平支撑层，再尝试风车布局。
        // 若风车布局不适用，则回退到普通有限宽度搜索。
        var onShelf = all.Where(x => x.Status == BoxStatus.OnShelf)
            .OrderByDescending(BoxBaseArea)
            .ThenBy(x => x.Order)
            .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
            .ToArray();
        var windmillBoxes = onShelf.Where(IsWindmillBox).ToArray();
        SearchState searched;
        if (windmillBoxes.Length >= 4 && windmillBoxes.Length % 4 == 0)
        {
            var otherBoxes = onShelf.Where(x => !IsWindmillBox(x)).ToArray();
            var windmillTypeKey = BoxTypeKey(windmillBoxes[0]);
            var currentTypeKey = GetOccupiedTypeKey(occupied, occupied.Count == 0
                ? -1
                : occupied.Max(x => x.Placement.LayerIndex));
            bool windmillFirst = otherBoxes.Length == 0
                || occupied.Count == 0
                || string.Equals(currentTypeKey, windmillTypeKey, StringComparison.Ordinal);

            if (windmillFirst)
            {
                var windmillOccupied = occupied.ToList();
                var windmillPlacements = new Dictionary<string, BoxPlacement>(StringComparer.Ordinal);
                bool windmillSucceeded = TryBuildWindmillGroups(windmillBoxes, windmillOccupied, windmillPlacements);
                if (windmillSucceeded)
                {
                    var otherSearch = SearchOnShelf(otherBoxes, windmillOccupied);
                    var combinedPlacements = new Dictionary<string, BoxPlacement>(
                        otherSearch.Placements, StringComparer.Ordinal);
                    foreach (var placement in windmillPlacements.Values)
                        combinedPlacements[placement.BoxNumber] = placement;
                    searched = otherSearch with { Placements = combinedPlacements };
                }
                else
                {
                    searched = SearchOnShelf(onShelf, occupied);
                }
            }
            else
            {
                // 当前层已有其他箱型时，先连续完成该箱型，再把风车箱放到后续层。
                var otherSearch = SearchOnShelf(otherBoxes, occupied.ToList());
                var combinedOccupied = otherSearch.Occupied.ToList();
                var windmillPlacements = new Dictionary<string, BoxPlacement>(StringComparer.Ordinal);
                if (TryBuildWindmillGroups(windmillBoxes, combinedOccupied, windmillPlacements))
                {
                    var combinedPlacements = new Dictionary<string, BoxPlacement>(
                        otherSearch.Placements, StringComparer.Ordinal);
                    foreach (var placement in windmillPlacements.Values)
                        combinedPlacements[placement.BoxNumber] = placement;
                    searched = otherSearch with
                    {
                        Occupied = combinedOccupied,
                        Placements = combinedPlacements,
                    };
                }
                else
                {
                    searched = SearchOnShelf(onShelf, occupied);
                }
            }
        }
        else
        {
            searched = SearchOnShelf(onShelf, occupied);
        }
        searched = NormalizeOnShelfOrders(searched, all);
        foreach (var placement in searched.Placements.Values)
        {
            output[placement.BoxNumber] = placement;
        }
        success &= searched.Placements.Count == onShelf.Length;

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

    /// <summary>
    /// 查询当前状态下最多还能完整放置多少个指定尺寸的箱子。
    /// 查询使用规划器副本，不会修改当前箱子集合、状态或最近一次规划结果。
    /// </summary>
    /// <param name="lengthMm">查询箱子的长度，单位为 mm。</param>
    /// <param name="widthMm">查询箱子的宽度，单位为 mm。</param>
    /// <param name="heightMm">查询箱子的高度，单位为 mm。</param>
    public RemainingCapacityResult GetMaxAdditionalBoxCount(
        double lengthMm,
        double widthMm,
        double heightMm)
    {
        var queryBox = new Box
        {
            BoxNumber = "__capacity_query__",
            LengthMm = lengthMm,
            WidthMm = widthMm,
            HeightMm = heightMm,
        };
        ValidateBox(queryBox);

        if (!CanFitSingleBox(queryBox))
        {
            return CreateCapacityResult(queryBox, false, 0, false, "查询箱子尺寸超出托盘可放置范围。");
        }

        var baseline = CreatePlanningCopy();
        var baselinePlan = baseline.GeneratePlan();
        int currentCount = _group.MutableBoxes.Count;
        if (!baselinePlan.PlanningResult || baselinePlan.Placements.Count != currentCount)
        {
            return CreateCapacityResult(queryBox, false, 0, false, "当前箱子规划未完成，无法查询剩余容量。");
        }

        double palletArea = (PalletXMaxMm - PalletXMinMm) * (PalletYMaxMm - PalletYMinMm);
        int layerUpperBound = (int)Math.Ceiling(StackMaxHeightMm / heightMm);
        int areaUpperBound = (int)Math.Ceiling(palletArea / (lengthMm * widthMm));
        int queryCount = Math.Max(1, areaUpperBound * layerUpperBound);
        var occupied = baselinePlan.Placements
            .Select(placement => ToPlacedBox(
                baseline.FindBox(placement.BoxNumber)!, placement, real: false))
            .ToList();
        int additional = 0;
        for (int index = 0; index < queryCount; index++)
        {
            var placement = baseline.GenerateCandidates(queryBox, occupied)
                .FirstOrDefault()?.Placement;
            if (placement is null)
                break;

            var placed = ToPlacedBox(queryBox, placement, real: false);
            if (!Fits(placed, occupied))
                break;
            occupied.Add(placed);
            additional++;
        }

        return CreateCapacityResult(queryBox, true, additional, true,
            additional == 0 ? "当前状态下无法再完整放置该尺寸箱子。" : "查询完成。");
    }

    private StackPlanner CreatePlanningCopy()
    {
        var copy = new StackPlanner();
        copy.AddBoxes(_group.MutableBoxes.Select(x => CloneBox(x) with { Status = BoxStatus.OnShelf }));
        var generated = copy.GeneratePlan();

        foreach (var box in _group.MutableBoxes.Where(x => x.Status != BoxStatus.OnShelf))
        {
            BoxPlacement? placement = null;
            if (_succeeded.TryGetValue(box.BoxNumber, out var fixedPlacement))
                placement = fixedPlacement.Placement;
            else if (_lastPlacements.TryGetValue(box.BoxNumber, out var lastPlacement))
                placement = lastPlacement;
            else
                placement = generated.Placements.FirstOrDefault(x => x.BoxNumber == box.BoxNumber);

            if (placement is null)
                throw new InvalidOperationException($"箱子 {box.BoxNumber} 没有可恢复的规划位置。");

            copy.UpdateBoxStatus(box.BoxNumber, box.Status, placement);
        }

        return copy;
    }

    private static bool CanFitSingleBox(Box box)
    {
        return new[] { (box.WidthMm, box.LengthMm), (box.LengthMm, box.WidthMm) }
            .Any(size => size.Item1 <= PalletXMaxMm - PalletXMinMm
                && size.Item2 <= PalletYMaxMm - PalletYMinMm)
            && box.HeightMm <= StackMaxHeightMm;
    }

    private static RemainingCapacityResult CreateCapacityResult(
        Box box,
        bool currentPlanValid,
        int additional,
        bool succeeded,
        string message) => new()
        {
            LengthMm = box.LengthMm,
            WidthMm = box.WidthMm,
            HeightMm = box.HeightMm,
            CurrentPlanValid = currentPlanValid,
            MaxAdditionalCount = additional,
            QuerySucceeded = succeeded,
            Message = message,
        };

    private bool TryBuildWindmillGroups(
        IReadOnlyList<Box> boxes,
        List<PlacedBox> occupied,
        IDictionary<string, BoxPlacement> placements)
    {
        if (boxes.Count < 4 || boxes.Count % 4 != 0)
            return false;

        int initialCount = occupied.Count;
        foreach (var group in boxes.Chunk(4))
        {
            var layer = TryBuildWindmillLayer(group, occupied);
            if (layer.Count != 4)
            {
                occupied.RemoveRange(initialCount, occupied.Count - initialCount);
                placements.Clear();
                return false;
            }
            foreach (var placement in layer.Values)
                placements[placement.BoxNumber] = placement;
        }
        return true;
    }

    private void AssignOrders()
    {
        // 非 OnShelf 箱子的 Order 是业务流程已经锁定的顺序，不能重新编号。
        // OnShelf 箱子严格按照加入集合的顺序分配顺序号，不再按箱型或箱号排序。
        var boxes = _group.MutableBoxes;
        if (boxes.Count > 0 && boxes.All(x => x.Status == BoxStatus.OnShelf))
        {
            int order = 0;
            foreach (var box in boxes)
            {
                box.Order = order++;
            }
            return;
        }
        var used = boxes.Where(x => x.Status != BoxStatus.OnShelf && x.Order >= 0).Select(x => x.Order).ToHashSet();
        int next = used.Count == 0 ? 0 : used.Max() + 1;
        foreach (var box in boxes.Where(x => x.Status == BoxStatus.OnShelf))
        {
            if (box.Order >= 0)
            {
                used.Add(box.Order);
                continue;
            }
            while (used.Contains(next)) next++;
            box.Order = next++;
            used.Add(box.Order);
        }
        if (boxes.Any(x => x.Status != BoxStatus.OnShelf && x.Order < 0))
            throw new InvalidOperationException("非 OnShelf 箱子必须已有固定 Order。");
    }

    private BoxPlacement? FindPlacement(Box box, List<PlacedBox> occupied)
    {
        return GenerateCandidates(box, occupied).FirstOrDefault()?.Placement;
    }

    /// <summary>
    /// 生成一个箱子的全部合法候选位置，并按固定规则排序。
    /// </summary>
    private IReadOnlyList<CandidatePlacement> GenerateCandidates(Box box, IReadOnlyList<PlacedBox> occupied)
    {
        // 候选点由托盘边界和已占用箱体的边界共同产生，避免连续浮点网格搜索。
        var candidates = new List<CandidatePlacement>();
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
            foreach (var x in xValues)
            foreach (var y in yValues)
            foreach (var baseZ in CandidateHeights(occupied))
            {
                // 候选必须同时满足边界、碰撞和支撑条件。
                var candidate = new Candidate(x, y, baseZ, width, length, box.HeightMm, orientation);
                if (!InsidePallet(candidate) || Collides(candidate, occupied)) continue;
                var supports = Supporting(candidate, occupied);
                if (baseZ > Epsilon && (supports.Count == 0 || !HasStableSupport(candidate, supports))) continue;
                // 托盘底层为第 0 层，叠放层级取支撑箱的最大层级加一。
                int layer = baseZ <= Epsilon ? 0 : supports.Max(x => x.Placement.LayerIndex) + 1;
                double supportArea = supports.Sum(x =>
                    OverlapLength(candidate.X - candidate.Width / 2, candidate.X + candidate.Width / 2, x.Left, x.Right)
                    * OverlapLength(candidate.Y - candidate.Length / 2, candidate.Y + candidate.Length / 2, x.Bottom, x.Top));
                var placement = new BoxPlacement
                {
                    Order = box.Order,
                    BoxNumber = box.BoxNumber,
                    Xmm = Round(candidate.X),
                    Ymm = Round(candidate.Y),
                    Zmm = Round(PlaceFloorZMm - candidate.BaseZ - box.HeightMm),
                    OrientationDeg = orientation,
                    LayerIndex = layer,
                };
                candidates.Add(new CandidatePlacement(placement, candidate with { Layer = layer }, supports.Count, supportArea));
            }
        }
        // 使用完整且固定的决胜顺序，不能依赖候选首次遍历顺序。
        return candidates.OrderBy(x => x.Placement.LayerIndex)
            .ThenBy(x => x.Candidate.BaseZ)
            .ThenByDescending(x => x.SupportArea)
            .ThenByDescending(x => x.SupportCount)
            .ThenBy(x => x.Placement.Ymm)
            .ThenBy(x => x.Placement.Xmm)
            .ThenBy(x => x.Placement.OrientationDeg)
            .ThenBy(x => x.Placement.BoxNumber, StringComparer.Ordinal)
            .Take(MaxCandidatesPerBox)
            .ToArray();
    }

    /// <summary>
    /// 返回允许尝试的箱底高度：托盘底面和所有已有箱子的顶部。
    /// 这样候选箱只能落在托盘或同高支撑面上，不会产生悬空位置。
    /// </summary>
    private static IEnumerable<double> CandidateHeights(IReadOnlyList<PlacedBox> occupied) =>
        new[] { 0d }.Concat(occupied.Select(x => x.BaseZ + x.Height)).Distinct().OrderBy(x => x);

    private static List<PlacedBox> Supporting(Candidate c, IReadOnlyList<PlacedBox> occupied)
    {
        // 只有 CanSupport=true 的箱顶才可作为规划支撑面。
        // 非 OnShelf 箱子虽然不一定是真实成功占用，但其规划占用可以提供支撑。
        return occupied.Where(x => x.TopZ <= c.BaseZ + Epsilon && Math.Abs(x.TopZ - c.BaseZ) <= Epsilon
                && x.CanSupport
                && OverlapLength(c.X - c.Width / 2, c.X + c.Width / 2, x.Left, x.Right) > Epsilon
                && OverlapLength(c.Y - c.Length / 2, c.Y + c.Length / 2, x.Bottom, x.Top) > Epsilon).ToList();
    }

    /// <summary>
    /// 判断箱子重心是否落在至少一个实际支撑箱的重叠区域内。
    /// 仅有边角或边缘交集虽然满足几何不碰撞，但在物理仿真中会产生倾倒。
    /// </summary>
    private static bool HasStableSupport(Candidate candidate, IReadOnlyList<PlacedBox> supports)
    {
        const double StabilityMarginMm = 5;
        double totalSupportArea = 0;
        double supportLeft = double.PositiveInfinity;
        double supportRight = double.NegativeInfinity;
        double supportBottom = double.PositiveInfinity;
        double supportTop = double.NegativeInfinity;
        foreach (var support in supports)
        {
            double overlapLeft = Math.Max(candidate.X - candidate.Width / 2, support.Left);
            double overlapRight = Math.Min(candidate.X + candidate.Width / 2, support.Right);
            double overlapBottom = Math.Max(candidate.Y - candidate.Length / 2, support.Bottom);
            double overlapTop = Math.Min(candidate.Y + candidate.Length / 2, support.Top);
            double overlapArea = Math.Max(0, overlapRight - overlapLeft)
                * Math.Max(0, overlapTop - overlapBottom);
            totalSupportArea += overlapArea;
            supportLeft = Math.Min(supportLeft, overlapLeft);
            supportRight = Math.Max(supportRight, overlapRight);
            supportBottom = Math.Min(supportBottom, overlapBottom);
            supportTop = Math.Max(supportTop, overlapTop);
            if (candidate.X >= overlapLeft + StabilityMarginMm
                && candidate.X <= overlapRight - StabilityMarginMm
                && candidate.Y >= overlapBottom + StabilityMarginMm
                && candidate.Y <= overlapTop - StabilityMarginMm)
                return true;
        }

        // 大箱子可能跨越下层箱子的间隙；此时重心不一定落在单个箱子内，
        // 但只要多个支撑箱的合计支撑面积充足且覆盖重心区域，仍属于稳定支撑。
        double candidateArea = candidate.Width * candidate.Length;
        if (totalSupportArea >= candidateArea * 0.5
            && candidate.X >= supportLeft + StabilityMarginMm
            && candidate.X <= supportRight - StabilityMarginMm
            && candidate.Y >= supportBottom + StabilityMarginMm
            && candidate.Y <= supportTop - StabilityMarginMm)
            return true;

        return false;
    }

    /// <summary>检查放置项是否在托盘边界内，并且没有与已有占用发生碰撞。</summary>
    private static bool Fits(PlacedBox item, IReadOnlyList<PlacedBox> occupied) => InsidePallet(item) && !Collides(item, occupied);
    private static bool InsidePallet(Candidate x) => x.X - x.Width / 2 >= PalletXMinMm - Epsilon && x.X + x.Width / 2 <= PalletXMaxMm + Epsilon
        && x.Y - x.Length / 2 >= PalletYMinMm - Epsilon && x.Y + x.Length / 2 <= PalletYMaxMm + Epsilon
        && x.BaseZ >= -Epsilon && x.BaseZ + x.Height <= StackMaxHeightMm + Epsilon;
    private static bool InsidePallet(PlacedBox x) => x.Left >= PalletXMinMm - Epsilon && x.Right <= PalletXMaxMm + Epsilon
        && x.Bottom >= PalletYMinMm - Epsilon && x.Top <= PalletYMaxMm + Epsilon && x.BaseZ >= -Epsilon
        && x.TopZ <= StackMaxHeightMm + Epsilon;
    // 候选箱的二维投影按间隙膨胀；只有 Z 方向存在重叠时才算三维碰撞。
    private static bool Collides(Candidate c, IReadOnlyList<PlacedBox> occupied) => occupied.Any(x =>
        c.BaseZ < x.TopZ - Epsilon && c.BaseZ + c.Height > x.BaseZ + Epsilon
        && OverlapLength(c.X - c.Width / 2 - StackBoxGapMm / 2, c.X + c.Width / 2 + StackBoxGapMm / 2, x.Left - StackBoxGapMm / 2, x.Right + StackBoxGapMm / 2) > Epsilon
        && OverlapLength(c.Y - c.Length / 2 - StackBoxGapMm / 2, c.Y + c.Length / 2 + StackBoxGapMm / 2, x.Bottom - StackBoxGapMm / 2, x.Top + StackBoxGapMm / 2) > Epsilon);
    // 恢复旧方案时使用同一套碰撞规则，保证规划占用与新候选的判断一致。
    private static bool Collides(PlacedBox c, IReadOnlyList<PlacedBox> occupied) => occupied.Any(x =>
        c.BaseZ < x.TopZ - Epsilon && c.TopZ > x.BaseZ + Epsilon
        && OverlapLength(c.Left - StackBoxGapMm / 2, c.Right + StackBoxGapMm / 2, x.Left - StackBoxGapMm / 2, x.Right + StackBoxGapMm / 2) > Epsilon
        && OverlapLength(c.Bottom - StackBoxGapMm / 2, c.Top + StackBoxGapMm / 2, x.Bottom - StackBoxGapMm / 2, x.Top + StackBoxGapMm / 2) > Epsilon);
    private static double OverlapLength(double a1, double a2, double b1, double b2) => Math.Max(0, Math.Min(a2, b2) - Math.Max(a1, b1));
    private static double Round(double value) => Math.Round(value, 3, MidpointRounding.AwayFromZero);

    /// <summary>
    /// 对仍在货架上的箱子执行确定性的有限宽度搜索。
    /// 搜索允许跳过当前箱子，从而在空间不足时尽量放置更多其他箱子。
    /// </summary>
    private SearchState SearchOnShelf(IReadOnlyList<Box> boxes, IReadOnlyList<PlacedBox> initialOccupied)
        => SearchOnShelf(boxes, initialOccupied,
            new Dictionary<string, BoxPlacement>(StringComparer.Ordinal));

    private SearchState SearchOnShelf(
        IReadOnlyList<Box> boxes,
        IReadOnlyList<PlacedBox> initialOccupied,
        IReadOnlyDictionary<string, BoxPlacement> initialPlacements)
    {
        var initial = new SearchState(
            boxes.Where(x => !initialPlacements.ContainsKey(x.BoxNumber)).ToArray(),
            initialOccupied.ToArray(),
            new Dictionary<string, BoxPlacement>(initialPlacements, StringComparer.Ordinal),
            -1,
            null);
        if (boxes.Count == 0) return initial;

        // 同一规划调用内，同箱型优先复用一层相对布局；模板只作为快路径，
        // 不能替代后续的边界、碰撞和支撑校验。
        var layerTemplates = new Dictionary<string, LayerTemplate>(StringComparer.Ordinal);
        var frontier = new[] { initial };
        SearchState best = initial;
        for (int depth = 0; depth < boxes.Count && frontier.Length > 0; depth++)
        {
            var next = new List<SearchState>();
            foreach (var state in frontier)
            {
                var choice = SelectNextBox(state);
                if (choice is null)
                {
                    // 当前分支的剩余箱子都没有合法候选位置，
                    // 将其作为终止分支保留，不能让一个死分支中断整个搜索。
                    next.Add(state);
                    continue;
                }
                var box = choice.Box;
                var remaining = state.Remaining.Where(x => !ReferenceEquals(x, box)).ToArray();
                var candidates = choice.Candidates;

                // 保留跳过分支，避免一个放不下的大箱子阻塞其他箱子。
                next.Add(new SearchState(remaining, state.Occupied, state.Placements,
                    state.PreferredLayer, state.PreferredTypeKey));
                foreach (var candidate in candidates)
                {
                    var placed = ToPlacedBox(box, candidate.Placement, false);
                    if (!Fits(placed, state.Occupied)) continue;
                    var placements = new Dictionary<string, BoxPlacement>(state.Placements, StringComparer.Ordinal)
                    {
                        [box.BoxNumber] = candidate.Placement,
                    };
                    next.Add(new SearchState(
                        remaining,
                        state.Occupied.Concat(new[] { placed }).ToArray(),
                        placements,
                        candidate.Placement.LayerIndex,
                        BoxTypeKey(box)));

                    // 模板批量分支只增加一个候选搜索状态；普通单箱分支始终保留，
                    // 因此模板不适用时不会改变原有规划能力。
                    string typeKey = BoxTypeKey(box);
                    if (!layerTemplates.TryGetValue(typeKey, out var template))
                    {
                        template = BuildLayerTemplate(box);
                        layerTemplates[typeKey] = template;
                    }
                    var bulkState = TryApplyLayerTemplate(
                        state, box, candidate, template, remaining);
                    if (bulkState is not null)
                        next.Add(bulkState);
                }
            }

            frontier = next.OrderByDescending(SearchPlacedCount)
                .ThenByDescending(x => SearchUpperBound(x))
                .ThenBy(SearchMaxLayer)
                .ThenBy(SearchMaxTopZ)
                .ThenByDescending(SearchLargeBoxLowLayerScore)
                .ThenBy(SearchBoundingArea)
                .ThenBy(SearchSignature, StringComparer.Ordinal)
                .Take(BeamWidth)
                .ToArray();

            var roundBest = frontier.OrderByDescending(SearchPlacedCount)
                .ThenBy(SearchMaxLayer)
                .ThenBy(SearchMaxTopZ)
                .ThenByDescending(SearchLargeBoxLowLayerScore)
                .ThenBy(SearchBoundingArea)
                .ThenBy(SearchSignature, StringComparer.Ordinal)
                .FirstOrDefault();
            if (roundBest is not null && IsBetterSearchState(roundBest, best)) best = roundBest;
            if (frontier.Any(x => x.Remaining.Count == 0 && x.Placements.Count == boxes.Count))
                break;
        }

        return frontier.Concat(new[] { best })
            .OrderByDescending(SearchPlacedCount)
            .ThenBy(SearchMaxLayer)
            .ThenBy(SearchMaxTopZ)
            .ThenByDescending(SearchLargeBoxLowLayerScore)
            .ThenBy(SearchBoundingArea)
            .ThenBy(SearchSignature, StringComparer.Ordinal)
            .First();
    }

    /// <summary>
    /// 使用代表箱子在空托盘上生成一个确定性的底层布局模板。
    /// 模板只保存同层相对位置，实际套用时仍需重新检查当前支撑面。
    /// </summary>
    private LayerTemplate BuildLayerTemplate(Box box)
    {
        var occupied = new List<PlacedBox>();
        var slots = new List<LayerTemplateSlot>();
        while (true)
        {
            var candidate = GenerateCandidates(box, occupied)
                .FirstOrDefault(x => x.Placement.LayerIndex == 0 && x.Candidate.BaseZ <= Epsilon);
            if (candidate is null) break;

            var placed = ToPlacedBox(box, candidate.Placement, false);
            if (!Fits(placed, occupied)) break;
            occupied.Add(placed);
            slots.Add(new LayerTemplateSlot(
                candidate.Placement.Xmm,
                candidate.Placement.Ymm,
                candidate.Placement.OrientationDeg));
        }

        if (slots.Count == 0)
            return new LayerTemplate(Array.Empty<LayerTemplateSlot>());

        var anchor = slots[0];
        return new LayerTemplate(slots.Select(x => new LayerTemplateSlot(
            x.RelativeX - anchor.RelativeX,
            x.RelativeY - anchor.RelativeY,
            x.OrientationDeg)).ToArray());
    }

    /// <summary>
    /// 将一层模板绑定到当前候选箱子，并批量生成同箱型箱子的搜索状态。
    /// 任一槽位不合法时不提交部分结果，由普通搜索分支继续处理。
    /// </summary>
    private SearchState? TryApplyLayerTemplate(
        SearchState state,
        Box anchorBox,
        CandidatePlacement anchor,
        LayerTemplate template,
        IReadOnlyList<Box> remaining)
    {
        if (template.Slots.Count < 2)
            return null;
        if (template.Slots[0].OrientationDeg != anchor.Placement.OrientationDeg)
            return null;

        var sameType = remaining
            .Where(x => string.Equals(BoxTypeKey(x), BoxTypeKey(anchorBox), StringComparison.Ordinal))
            .OrderBy(x => x.Order)
            .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
            .Take(template.Slots.Count - 1)
            .ToArray();
        if (sameType.Length == 0)
            return null;

        var occupied = state.Occupied.ToList();
        var placements = new Dictionary<string, BoxPlacement>(state.Placements, StringComparer.Ordinal)
        {
            [anchorBox.BoxNumber] = anchor.Placement,
        };
        occupied.Add(ToPlacedBox(anchorBox, anchor.Placement, false));
        var usedNumbers = new HashSet<string>(StringComparer.Ordinal) { anchorBox.BoxNumber };

        for (int index = 0; index < sameType.Length; index++)
        {
            var box = sameType[index];
            var slot = template.Slots[index + 1];
            var candidate = new Candidate(
                anchor.Placement.Xmm + slot.RelativeX,
                anchor.Placement.Ymm + slot.RelativeY,
                anchor.Candidate.BaseZ,
                slot.OrientationDeg == 0 ? box.WidthMm : box.LengthMm,
                slot.OrientationDeg == 0 ? box.LengthMm : box.WidthMm,
                box.HeightMm,
                slot.OrientationDeg);

            if (!InsidePallet(candidate) || Collides(candidate, occupied))
                return null;

            var supports = Supporting(candidate, occupied);
            if (candidate.BaseZ > Epsilon
                && (supports.Count == 0 || !HasStableSupport(candidate, supports)))
                return null;

            int layer = candidate.BaseZ <= Epsilon
                ? 0
                : supports.Max(x => x.Placement.LayerIndex) + 1;
            if (layer != anchor.Placement.LayerIndex)
                return null;

            var placement = new BoxPlacement
            {
                Order = box.Order,
                BoxNumber = box.BoxNumber,
                Xmm = Round(candidate.X),
                Ymm = Round(candidate.Y),
                Zmm = Round(PlaceFloorZMm - candidate.BaseZ - box.HeightMm),
                OrientationDeg = slot.OrientationDeg,
                LayerIndex = layer,
            };
            var placed = ToPlacedBox(box, placement, false);
            if (!Fits(placed, occupied))
                return null;

            occupied.Add(placed);
            placements[box.BoxNumber] = placement;
            usedNumbers.Add(box.BoxNumber);
        }

        var nextRemaining = remaining.Where(x => !usedNumbers.Contains(x.BoxNumber)).ToArray();
        return new SearchState(
            nextRemaining,
            occupied.ToArray(),
            placements,
            anchor.Placement.LayerIndex,
            BoxTypeKey(anchorBox));
    }

    /// <summary>
    /// 为四个 400×600 箱子生成同一支撑高度的风车布局。
    /// 支撑层可以是托盘底面，也可以是已经铺平的上一层。
    /// </summary>
    private static Dictionary<string, BoxPlacement> TryBuildWindmillLayer(
        IReadOnlyList<Box> boxes,
        List<PlacedBox> occupied)
    {
        var result = new Dictionary<string, BoxPlacement>(StringComparer.Ordinal);
        if (boxes.Count < 4)
            return result;

        var large = boxes.Take(4).ToArray();
        if (large.Any(x => !IsWindmillBox(x)))
            return result;

        double xMin = PalletXMinMm;
        double yMin = PalletYMinMm;
        var candidates = new[]
        {
            (X: xMin + 200, Y: yMin + 300, Angle: 90),
            (X: xMin + 300, Y: yMin + 820, Angle: 0),
            (X: xMin + 720, Y: yMin + 280, Angle: 0),
            (X: xMin + 820, Y: yMin + 800, Angle: 90),
        };

        foreach (var baseZ in CandidateHeights(occupied).OrderByDescending(x => x))
        {
            result.Clear();
            int initialCount = occupied.Count;
            int layer = baseZ <= Epsilon
                ? 0
                : -1;
            bool valid = true;
            for (int i = 0; i < large.Length; i++)
            {
                var candidate = candidates[i];
                var geometry = new Candidate(
                    candidate.X, candidate.Y, baseZ,
                    candidate.Angle == 0 ? large[i].WidthMm : large[i].LengthMm,
                    candidate.Angle == 0 ? large[i].LengthMm : large[i].WidthMm,
                    large[i].HeightMm, candidate.Angle);
                var supports = Supporting(geometry, occupied);
                if (!InsidePallet(geometry)
                    || Collides(geometry, occupied)
                    || baseZ > Epsilon && (supports.Count == 0 || !HasStableSupport(geometry, supports)))
                {
                    valid = false;
                    break;
                }

                int candidateLayer = baseZ <= Epsilon
                    ? 0
                    : supports.Max(x => x.Placement.LayerIndex) + 1;
                layer = layer < 0 ? candidateLayer : layer;
                if (candidateLayer != layer)
                {
                    valid = false;
                    break;
                }

                var placement = new BoxPlacement
                {
                    Order = large[i].Order,
                    BoxNumber = large[i].BoxNumber,
                    Xmm = candidate.X,
                    Ymm = candidate.Y,
                    Zmm = PlaceFloorZMm - baseZ - large[i].HeightMm,
                    OrientationDeg = candidate.Angle,
                    LayerIndex = layer,
                };
                var placed = ToPlacedBox(large[i], placement, false);
                if (!Fits(placed, occupied))
                {
                    valid = false;
                    break;
                }
                occupied.Add(placed);
                result[large[i].BoxNumber] = placement;
            }

            if (valid && result.Count == 4)
                return result;

            occupied.RemoveRange(initialCount, occupied.Count - initialCount);
            result.Clear();
        }

        return result;
    }

    private static bool IsWindmillBox(Box box) =>
        (Math.Abs(box.LengthMm - 400) <= Epsilon && Math.Abs(box.WidthMm - 600) <= Epsilon)
        || (Math.Abs(box.LengthMm - 600) <= Epsilon && Math.Abs(box.WidthMm - 400) <= Epsilon);

    /// <summary>
    /// 将搜索结果中的 OnShelf 箱子恢复为加入时分配的顺序号，
    /// 同时避开非 OnShelf 箱子已经锁定的顺序号。
    /// </summary>
    private static SearchState NormalizeOnShelfOrders(SearchState state, IReadOnlyList<Box> all)
    {
        var orders = all.ToDictionary(x => x.BoxNumber, x => x.Order, StringComparer.Ordinal);
        var replacements = new Dictionary<string, BoxPlacement>(StringComparer.Ordinal);
        var boxes = all.ToDictionary(x => x.BoxNumber, StringComparer.Ordinal);
        var usedOrders = all.Where(x => x.Status != BoxStatus.OnShelf && x.Order >= 0)
            .Select(x => x.Order)
            .ToHashSet();
        int nextOrder = 0;
        foreach (var placement in state.Placements.Values
                     .OrderBy(x => x.LayerIndex)
                     // 参考 SKQ：同层按 X 从大到小、同列按 Y 从大到小下发，
                     // 让机械臂沿固定方向逐列放置，避免同层顺序看起来无序。
                     .ThenByDescending(x => x.Xmm)
                     .ThenByDescending(x => x.Ymm)
                     .ThenByDescending(x => BoxBaseArea(boxes[x.BoxNumber]))
                     .ThenBy(x => BoxTypeKey(boxes[x.BoxNumber]), StringComparer.Ordinal)
                     .ThenBy(x => orders[x.BoxNumber])
                     .ThenBy(x => x.BoxNumber, StringComparer.Ordinal))
        {
            while (usedOrders.Contains(nextOrder)) nextOrder++;
            replacements[placement.BoxNumber] = placement with { Order = nextOrder++ };
            usedOrders.Add(replacements[placement.BoxNumber].Order);
        }
        return state with { Placements = replacements };
    }

    /// <summary>
    /// 选择当前层的下一个箱子。只有当前最低可用层没有任何剩余箱子可放时，
    /// 才允许搜索进入更高层；当前层优先继续放置同一箱型。
    /// </summary>
    private BoxChoice? SelectNextBox(SearchState state)
    {
        // 候选几何只由箱子尺寸、朝向和当前占用决定，与箱号及 Order 无关。
        // 相同箱型只为确定性代表箱生成一次候选，避免 128 个相同箱子在每个搜索状态
        // 中重复执行完全相同的边界、碰撞和支撑计算。
        var candidatesByBox = state.Remaining
            .GroupBy(BoxTypeKey, StringComparer.Ordinal)
            .Select(group =>
            {
                var box = group
                    .OrderBy(x => x.Order)
                    .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
                    .First();
                return new SearchBoxCandidates(box, GenerateCandidates(box, state.Occupied));
            })
            .ToArray();
        int currentLayer = candidatesByBox
            .SelectMany(x => x.Candidates)
            .Select(x => x.Placement.LayerIndex)
            .DefaultIfEmpty(int.MaxValue)
            .Min();

        var currentLayerBoxes = candidatesByBox
            .Select(x => new SearchBoxCandidates(x.Box,
                x.Candidates.Where(c => c.Placement.LayerIndex == currentLayer).ToArray()))
            .Where(x => x.Candidates.Count > 0)
            .ToArray();
        if (currentLayerBoxes.Length == 0)
            return null;

        string? typeKey = state.PreferredLayer == currentLayer
            ? state.PreferredTypeKey
            : GetOccupiedTypeKey(state.Occupied, currentLayer);
        var preferredTypeBoxes = string.IsNullOrEmpty(typeKey)
            ? Array.Empty<SearchBoxCandidates>()
            : currentLayerBoxes.Where(x => BoxTypeKey(x.Box) == typeKey).ToArray();
        var selectable = preferredTypeBoxes.Length > 0 ? preferredTypeBoxes : currentLayerBoxes;

        var selected = selectable
            .GroupBy(x => BoxTypeKey(x.Box), StringComparer.Ordinal)
            // 当前最低层优先放底面积最大的箱型，避免数量多的小箱子抢占底层。
            .OrderByDescending(group => group.Max(x => BoxBaseArea(x.Box)))
            .ThenByDescending(group => group.Count())
            .ThenByDescending(group => group.Sum(x => x.Candidates.Count))
            .ThenBy(group => group.Key, StringComparer.Ordinal)
            .First()
            .OrderBy(x => x.Candidates.Count)
            .ThenByDescending(x => BoxBaseArea(x.Box))
            .ThenByDescending(x => BoxVolume(x.Box))
            .ThenByDescending(x => x.Box.HeightMm)
            .ThenBy(x => x.Box.BoxNumber, StringComparer.Ordinal)
            .First();

        return new BoxChoice(selected.Box, selected.Candidates);
    }

    private string? GetOccupiedTypeKey(IReadOnlyList<PlacedBox> occupied, int layer)
    {
        if (layer < 0)
            return null;

        return occupied
            .Where(x => x.Placement.LayerIndex == layer)
            .Select(x => FindBox(x.Placement.BoxNumber))
            .Where(x => x is not null)
            .Select(x => BoxTypeKey(x!))
            .GroupBy(x => x, StringComparer.Ordinal)
            .OrderByDescending(x => x.Count())
            .ThenBy(x => x.Key, StringComparer.Ordinal)
            .Select(x => x.Key)
            .FirstOrDefault();
    }

    private static string BoxTypeKey(Box box)
    {
        double first = Math.Min(box.LengthMm, box.WidthMm);
        double second = Math.Max(box.LengthMm, box.WidthMm);
        return string.Join("x", first.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            second.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            box.HeightMm.ToString("R", System.Globalization.CultureInfo.InvariantCulture));
    }

    private static int SearchPlacedCount(SearchState state) => state.Placements.Count;
    private static int SearchUpperBound(SearchState state) => state.Placements.Count + state.Remaining.Count;
    private static int SearchMaxLayer(SearchState state) => state.Occupied.Count == 0 ? 0 : state.Occupied.Max(x => x.Placement.LayerIndex);
    private static double SearchMaxTopZ(SearchState state) => state.Occupied.Count == 0 ? 0 : state.Occupied.Max(x => x.TopZ);
    private static double SearchLargeBoxLowLayerScore(SearchState state) => state.Occupied.Sum(x =>
        BoxBaseAreaFromPlaced(x) / (1 + x.Placement.LayerIndex));
    private static double SearchBoundingArea(SearchState state)
    {
        if (state.Occupied.Count == 0) return 0;
        double left = state.Occupied.Min(x => x.Left);
        double right = state.Occupied.Max(x => x.Right);
        double bottom = state.Occupied.Min(x => x.Bottom);
        double top = state.Occupied.Max(x => x.Top);
        return (right - left) * (top - bottom);
    }

    private static string SearchSignature(SearchState state) => string.Join(";", state.Placements.Values
        .OrderBy(x => x.BoxNumber, StringComparer.Ordinal)
        .Select(x => string.Join("|", x.BoxNumber, x.Xmm.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            x.Ymm.ToString("R", System.Globalization.CultureInfo.InvariantCulture), x.Zmm.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            x.OrientationDeg, x.LayerIndex)));

    private static bool IsBetterSearchState(SearchState candidate, SearchState current) =>
        SearchPlacedCount(candidate) > SearchPlacedCount(current)
        || SearchPlacedCount(candidate) == SearchPlacedCount(current)
        && (SearchMaxLayer(candidate) < SearchMaxLayer(current)
            || SearchMaxLayer(candidate) == SearchMaxLayer(current) && SearchMaxTopZ(candidate) < SearchMaxTopZ(current)
            || SearchMaxLayer(candidate) == SearchMaxLayer(current) && SearchMaxTopZ(candidate).Equals(SearchMaxTopZ(current))
            && SearchLargeBoxLowLayerScore(candidate) > SearchLargeBoxLowLayerScore(current));

    private static double BoxBaseArea(Box box) => box.LengthMm * box.WidthMm;
    private static double BoxVolume(Box box) => BoxBaseArea(box) * box.HeightMm;
    private static double BoxBaseAreaFromPlaced(PlacedBox box) => box.Width * box.Length;

    /// <summary>
    /// 将公开的机械 Z 坐标反算为内部箱底高度 BaseZ，供边界、碰撞和支撑计算使用。
    /// </summary>
    private static PlacedBox ToPlacedBox(Box box, BoxPlacement placement, bool real)
    {
        bool rotated = placement.OrientationDeg == 90;
        double width = rotated ? box.LengthMm : box.WidthMm;
        double length = rotated ? box.WidthMm : box.LengthMm;
        double baseZ = PlaceFloorZMm - placement.Zmm - box.HeightMm;
        // 规划阶段已经写入 occupied 的箱子都可以作为后续候选的支撑面，
        // 包括仍处于 OnShelf 状态但已经被本次方案放置的箱子。
        // real 仅表示是否为真实成功占用，不限制规划支撑能力。
        return new PlacedBox(placement.Xmm, placement.Ymm, baseZ, width, length, box.HeightMm, placement,
            real, true);
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
        int Orientation, int Layer = 0);
    /// <summary>单个箱子的合法候选位置及其内部评分。</summary>
    private sealed record CandidatePlacement(BoxPlacement Placement, Candidate Candidate, int SupportCount, double SupportArea);
    /// <summary>有限宽度搜索的一层布局状态。</summary>
    private sealed record SearchState(IReadOnlyList<Box> Remaining, IReadOnlyList<PlacedBox> Occupied,
        IReadOnlyDictionary<string, BoxPlacement> Placements, int PreferredLayer, string? PreferredTypeKey);

    private sealed record SearchBoxCandidates(Box Box, IReadOnlyList<CandidatePlacement> Candidates);
    private sealed record BoxChoice(Box Box, IReadOnlyList<CandidatePlacement> Candidates);
    /// <summary>同箱型一层布局的相对槽位。</summary>
    private sealed record LayerTemplateSlot(double RelativeX, double RelativeY, int OrientationDeg);
    /// <summary>同箱型一层布局模板，不包含具体箱号和绝对高度。</summary>
    private sealed record LayerTemplate(IReadOnlyList<LayerTemplateSlot> Slots);
}
