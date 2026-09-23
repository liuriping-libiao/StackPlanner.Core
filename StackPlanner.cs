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
    private const int BeamWidth = 96;
    /// <summary>单个箱子最多保留的候选位置数量。</summary>
    private const int MaxCandidatesPerBox = 64;
    private double _palletXMinMm;
    private double _palletXMaxMm;
    private double _palletYMinMm;
    private double _palletYMaxMm;
    private bool _hasGeneratedPlan;
    /// <summary>DLL 当前维护的全部箱子。</summary>
    private readonly BoxGroup _group = new();
    /// <summary>已实际堆垛成功箱子的固定位置，不允许普通重规划改变。</summary>
    private readonly Dictionary<string, FixedPlacement> _succeeded = new(StringComparer.Ordinal);
    /// <summary>最近一次完整规划，用于恢复非 OnShelf 箱子的规划占用。</summary>
    private readonly Dictionary<string, BoxPlacement> _lastPlacements = new(StringComparer.Ordinal);
    /// <summary>已删除但等待同规格箱子补位的次数，仅供利用率规划使用。</summary>
    private readonly Dictionary<string, int> _pendingReplacementCounts = new(StringComparer.Ordinal);
    /// <summary>匹配删除记录、等待在利用率规划中追加到同规格末尾的新箱号。</summary>
    private readonly Dictionary<string, HashSet<string>> _pendingReplacementBoxes = new(StringComparer.Ordinal);
    /// <summary>删除后保留的、可供同规格新箱复用的规划位置。</summary>
    private readonly Dictionary<string, List<BoxPlacement>> _reusableUtilizationPlacements = new(StringComparer.Ordinal);
    /// <summary>对外返回的最近一次规划结果。</summary>
    private StackPlanResult _lastPlan = new() { Placements = Array.Empty<BoxPlacement>(), PlanningResult = true };

    /// <summary>
    /// 使用默认托盘边界创建规划器。
    /// 托盘边界在实例创建后不可修改，保证一次任务内规划结果的坐标系稳定。
    /// </summary>
    public StackPlanner()
        : this(PalletXMinMm, PalletXMaxMm, PalletYMinMm, PalletYMaxMm)
    {
    }

    /// <summary>
    /// 使用指定托盘边界创建规划器。边界单位为 mm，且必须在首次规划前确定。
    /// </summary>
    public StackPlanner(
        double palletXMinMm,
        double palletXMaxMm,
        double palletYMinMm,
        double palletYMaxMm)
    {
        ValidatePalletDimensions(palletXMinMm, palletXMaxMm, palletYMinMm, palletYMaxMm);

        _palletXMinMm = palletXMinMm;
        _palletXMaxMm = palletXMaxMm;
        _palletYMinMm = palletYMinMm;
        _palletYMaxMm = palletYMaxMm;
    }

    private static void ValidatePalletDimensions(
        double palletXMinMm,
        double palletXMaxMm,
        double palletYMinMm,
        double palletYMaxMm)
    {
        if (!double.IsFinite(palletXMinMm) || !double.IsFinite(palletXMaxMm)
            || !double.IsFinite(palletYMinMm) || !double.IsFinite(palletYMaxMm)
            || palletXMaxMm <= palletXMinMm || palletYMaxMm <= palletYMinMm)
            throw new ArgumentException("托盘边界必须为有限数值，且最大值必须大于最小值。");
    }

    /// <summary>当前规划器使用的托盘 X 最小边界，单位为 mm。</summary>
    public double PalletXMin => _palletXMinMm;
    /// <summary>当前规划器使用的托盘 X 最大边界，单位为 mm。</summary>
    public double PalletXMax => _palletXMaxMm;
    /// <summary>当前规划器使用的托盘 Y 最小边界，单位为 mm。</summary>
    public double PalletYMin => _palletYMinMm;
    /// <summary>当前规划器使用的托盘 Y 最大边界，单位为 mm。</summary>
    public double PalletYMax => _palletYMaxMm;

    /// <summary>
    /// 设置当前任务的托盘边界。必须在首次成功生成规划前调用，规划完成后不能修改。
    /// </summary>
    /// <param name="palletXMinMm">托盘 X 最小边界，单位为 mm。</param>
    /// <param name="palletXMaxMm">托盘 X 最大边界，单位为 mm。</param>
    /// <param name="palletYMinMm">托盘 Y 最小边界，单位为 mm。</param>
    /// <param name="palletYMaxMm">托盘 Y 最大边界，单位为 mm。</param>
    /// <exception cref="InvalidOperationException">规划已经生成，不能修改托盘尺寸。</exception>
    /// <exception cref="ArgumentException">托盘边界不是有限值或最大值不大于最小值。</exception>
    public void SetPalletDimensions(
        double palletXMinMm,
        double palletXMaxMm,
        double palletYMinMm,
        double palletYMaxMm)
    {
        if (_hasGeneratedPlan)
            throw new InvalidOperationException("规划已经生成，不能修改托盘尺寸。");

        ValidatePalletDimensions(palletXMinMm, palletXMaxMm, palletYMinMm, palletYMaxMm);
        _palletXMinMm = palletXMinMm;
        _palletXMaxMm = palletXMaxMm;
        _palletYMinMm = palletYMinMm;
        _palletYMaxMm = palletYMaxMm;
    }

    /// <summary>
    /// 获取当前箱子集合的副本。调用方修改返回对象不会影响 DLL 内部状态。
    /// </summary>
    public IReadOnlyList<Box> GetBoxes() => _group.Boxes.Select(CloneBox).ToArray();

    /// <summary>获取最近一次规划结果的副本。</summary>
    public StackPlanResult CurrentPlan => ClonePlan(_lastPlan);

    /// <summary>生成当前箱子集合和规划数据的不可变快照。</summary>
    public PlannerSnapshot CreateSnapshot() => new()
    {
        Revision = 0,
        UpdatedAtUtc = DateTimeOffset.UtcNow,
        Boxes = GetBoxes(),
        Plan = CurrentPlan,
        ReusablePlacements = ExportReusablePlacements(),
    };

    /// <summary>将当前完整状态原子发布到本地快照文件。</summary>
    public PlannerSnapshot SaveSnapshot(string filePath)
        => PlannerSnapshotStore.Save(filePath, CreateSnapshot());

    /// <summary>
    /// 从完整快照恢复 Core 状态。恢复前会校验箱号、箱子属性和规划引用关系。
    /// </summary>
    public void RestoreSnapshot(PlannerSnapshot snapshot)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        if (snapshot.SchemaVersion != 1)
            throw new InvalidDataException($"不支持的规划快照版本：{snapshot.SchemaVersion}");

        var boxes = snapshot.Boxes.Select(CloneBox).ToArray();
        var numbers = new HashSet<string>(StringComparer.Ordinal);
        foreach (var box in boxes)
        {
            ValidateBox(box);
            if (!numbers.Add(box.BoxNumber))
                throw new InvalidDataException($"规划快照包含重复箱号：{box.BoxNumber}");
        }

        var placements = snapshot.Plan.Placements.Select(ClonePlacement).ToArray();
        if (placements.Any(x => !numbers.Contains(x.BoxNumber))
            || placements.Select(x => x.BoxNumber).Distinct(StringComparer.Ordinal).Count() != placements.Length)
            throw new InvalidDataException("规划快照中的放置结果包含未知或重复箱号。");

        _group.MutableBoxes.Clear();
        _group.MutableBoxes.AddRange(boxes);
        _succeeded.Clear();
        _lastPlacements.Clear();
        _pendingReplacementCounts.Clear();
        _pendingReplacementBoxes.Clear();
        _reusableUtilizationPlacements.Clear();

        foreach (var placement in placements)
            _lastPlacements[placement.BoxNumber] = placement;
        foreach (var box in boxes.Where(x => x.Status == BoxStatus.StackingSucceeded))
        {
            if (!_lastPlacements.TryGetValue(box.BoxNumber, out var placement))
                throw new InvalidDataException($"堆垛成功箱子缺少固定位置：{box.BoxNumber}");
            _succeeded[box.BoxNumber] = new FixedPlacement(
                ClonePlacement(placement), box.LengthMm, box.WidthMm, box.HeightMm);
        }
        foreach (var reusableSet in snapshot.ReusablePlacements)
        {
            string key = BoxTypeKey(new Box
            {
                BoxNumber = "snapshot-dimension",
                LengthMm = reusableSet.Dimension.LengthMm,
                WidthMm = reusableSet.Dimension.WidthMm,
                HeightMm = reusableSet.Dimension.HeightMm,
            });
            _reusableUtilizationPlacements[key] = reusableSet.Placements
                .Select(ClonePlacement)
                .ToList();
        }

        _lastPlan = new StackPlanResult
        {
            Placements = placements,
            PlanningResult = snapshot.Plan.PlanningResult,
        };
        _hasGeneratedPlan = placements.Length > 0 || boxes.Length == 0;
    }

    /// <summary>从本地快照文件恢复 Core 状态。</summary>
    public void LoadSnapshot(string filePath) => RestoreSnapshot(PlannerSnapshotStore.Load(filePath));

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
        var added = CloneBox(box);
        RegisterReplacementBox(added);
        _group.MutableBoxes.Add(added);
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
        foreach (var box in incoming)
        {
            RegisterReplacementBox(box);
            _group.MutableBoxes.Add(box);
        }
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
        string typeKey = BoxTypeKey(box);
        _group.MutableBoxes.Remove(box);
        foreach (var following in _group.MutableBoxes.Where(x => x.Order > removedOrder))
            following.Order--;
        _pendingReplacementCounts[typeKey] = _pendingReplacementCounts.GetValueOrDefault(typeKey) + 1;
        _succeeded.Remove(boxNumber);
        _lastPlacements.Remove(boxNumber);
        return true;
    }

    /// <summary>
    /// 不重新规划地增加一个箱子。
    /// 新箱子只能使用此前减箱释放的同尺寸规划位置；没有可复用位置时不会增加箱子。
    /// </summary>
    /// <param name="box">待增加的箱子，必须处于 OnShelf 状态。</param>
    /// <returns>增加结果，以及操作后的完整箱子集合和规划快照。</returns>
    public UtilizationMutationResult AddBoxWithoutReplanning(Box box)
    {
        ArgumentNullException.ThrowIfNull(box);
        ValidateBox(box);

        if (box.Status != BoxStatus.OnShelf)
            return MutationFailure("新增箱子必须处于 OnShelf 状态。");
        if (_group.MutableBoxes.Any(x => string.Equals(x.BoxNumber, box.BoxNumber, StringComparison.Ordinal)))
            return MutationFailure($"箱号已存在：{box.BoxNumber}。");
        if (!_lastPlan.PlanningResult || _lastPlacements.Count != _group.MutableBoxes.Count)
            return MutationFailure("当前没有完整有效的利用率规划，不能不重新规划地加箱。");

        string key = BoxTypeKey(box);
        if (!_reusableUtilizationPlacements.TryGetValue(key, out var reusable)
            || reusable.Count == 0)
            return MutationFailure($"尺寸 {key} 没有可复用的规划位置，不能不重新规划地加箱。");

        var added = CloneBox(box);
        var source = reusable.MinBy(point => point.Order)!;
        reusable.Remove(source);
        _group.MutableBoxes.Add(added);

        var placement = source with
        {
            BoxNumber = added.BoxNumber,
        };
        added.Order = placement.Order;
        _lastPlacements[added.BoxNumber] = placement;
        _lastPlan = new StackPlanResult
        {
            Placements = _lastPlan.Placements
                .Append(placement)
                .OrderBy(point => point.Order)
                .ToArray(),
            PlanningResult = true,
        };
        _hasGeneratedPlan = true;
        return MutationSuccess("不重新规划地加箱完成。");
    }

    /// <summary>
    /// 不重新规划地删除一个箱子。
    /// 删除后，同尺寸箱子依次使用前一个箱子的规划位置，并保留最后释放的位置供加箱复用。
    /// </summary>
    /// <param name="box">待删除的箱子；仅使用其 BoxNumber 定位 Core 内部箱子。</param>
    /// <returns>删除结果，以及操作后的完整箱子集合和规划快照。</returns>
    public UtilizationMutationResult RemoveBoxWithoutReplanning(Box box)
    {
        ArgumentNullException.ThrowIfNull(box);
        var existing = FindBox(box.BoxNumber);
        if (existing is null)
            return MutationFailure($"箱号不存在：{box.BoxNumber}。");
        if (existing.Status is BoxStatus.Stacking or BoxStatus.StackingSucceeded)
            return MutationFailure("堆垛中或堆垛成功的箱子不能删除。");
        if (!_lastPlan.PlanningResult || _lastPlacements.Count != _group.MutableBoxes.Count)
            return MutationFailure("当前没有完整有效的利用率规划，不能不重新规划地减箱。");

        RemoveUtilizationBox(existing.BoxNumber);
        return MutationSuccess("不重新规划地减箱完成。");
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
        _pendingReplacementCounts.Clear();
        _pendingReplacementBoxes.Clear();
        _lastPlan = new StackPlanResult { Placements = Array.Empty<BoxPlacement>(), PlanningResult = true };
        _hasGeneratedPlan = false;
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

        // OnShelf 箱子允许重排：逐层寻找空间利用率最高的同高混合布局。
        var onShelf = all.Where(x => x.Status == BoxStatus.OnShelf)
            .OrderByDescending(BoxBaseArea)
            .ThenBy(x => x.Order)
            .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
            .ToArray();
        SearchState searched = SearchOnShelf(onShelf, occupied);
        searched = NormalizeOnShelfOrders(searched, all);
        foreach (var placement in searched.Placements.Values)
        {
            output[placement.BoxNumber] = placement;
        }
        success &= searched.Placements.Count == onShelf.Length;

        var placements = output.Values.OrderBy(x => x.Order).ThenBy(x => x.BoxNumber, StringComparer.Ordinal).ToArray();
        placements = NormalizePlacementOrders(placements);
        foreach (var placement in placements)
        {
            var box = FindBox(placement.BoxNumber)!;
            box.Order = placement.Order;
        }
        _lastPlacements.Clear();
        foreach (var placement in placements) _lastPlacements[placement.BoxNumber] = ClonePlacement(placement);
        _lastPlan = new StackPlanResult { Placements = placements, PlanningResult = success && placements.Length == all.Length };
        _hasGeneratedPlan = true;
        return ClonePlan(_lastPlan);
    }

    /// <summary>
    /// 仅根据候选尺寸生成逐层空间利用率规划。
    /// 不接收库存数量；每种尺寸视为可无限供应。结果同时返回箱子数量和空间利用率。
    /// </summary>
    public UtilizationPlanResult PlanBestUtilization(IEnumerable<BoxDimension> dimensions)
    {
        ArgumentNullException.ThrowIfNull(dimensions);
        var input = dimensions.ToArray();
        if (input.Length == 0)
            return EmptyUtilizationPlan("至少需要一个箱子尺寸。");

        foreach (var dimension in input)
        {
            if (dimension is null || !double.IsFinite(dimension.LengthMm)
                || !double.IsFinite(dimension.WidthMm) || !double.IsFinite(dimension.HeightMm)
                || dimension.LengthMm <= 0 || dimension.WidthMm <= 0 || dimension.HeightMm <= 0
                || dimension.MinimumCount < 0)
                return EmptyUtilizationPlan("箱子尺寸必须为有限正数，最小数量不能小于 0。");
        }

        var unique = input
            .GroupBy(x => DimensionKey(x), StringComparer.Ordinal)
            .Select(group =>
            {
                var first = group.First();
                return first with { MinimumCount = group.Max(x => x.MinimumCount) };
            })
            .OrderBy(x => DimensionKey(x), StringComparer.Ordinal)
            .ToArray();

        // 直接搜索真实二维布局，不再生成有限数量的临时箱子。
        var candidates = unique
            .GroupBy(x => x.HeightMm)
            .SelectMany(group => TypeCombinations(
                group.OrderBy(x => DimensionKey(x), StringComparer.Ordinal).ToArray(), 3))
            .Select(FindBestDirectLayer)
            .Where(x => x is not null)
            .Cast<DirectLayerResult>()
            .ToArray();
        if (candidates.Length == 0)
            return EmptyUtilizationPlan("没有尺寸可以形成合法布局。");

        var selected = FindBestLayerStack(candidates, unique);
        if (selected is null)
            return EmptyUtilizationPlan("在托盘尺寸和最大堆垛高度限制下，无法满足箱型最小数量约束。", false);

        var layers = new List<LayerUtilizationSummary>();
        var selectedCounts = selected.Layers
            .SelectMany(x => x.Placements)
            .GroupBy(x => DimensionKey(x.Dimension), StringComparer.Ordinal)
            .ToDictionary(group => group.Key, group => group.Count(), StringComparer.Ordinal);
        var plannedCounts = unique
            .Select(dimension => new BoxTypeCount
            {
                LengthMm = dimension.LengthMm,
                WidthMm = dimension.WidthMm,
                HeightMm = dimension.HeightMm,
                Count = selectedCounts.GetValueOrDefault(DimensionKey(dimension)),
            })
            .OrderBy(x => DimensionKey(new BoxDimension
            {
                LengthMm = x.LengthMm,
                WidthMm = x.WidthMm,
                HeightMm = x.HeightMm,
            }), StringComparer.Ordinal)
            .ToArray();
        double palletArea = (_palletXMaxMm - _palletXMinMm) * (_palletYMaxMm - _palletYMinMm);
        for (int layerIndex = 0; layerIndex < selected.Layers.Count; layerIndex++)
        {
            var layer = selected.Layers[layerIndex];
            layers.Add(new LayerUtilizationSummary
            {
                LayerIndex = layerIndex,
                Utilization = layer.OccupiedArea / palletArea,
            });
        }

        double utilization = layers.Count == 0 ? 0 : layers.Average(x => x.Utilization);
        return new UtilizationPlanResult
        {
            PlannedCounts = plannedCounts,
            Layers = layers,
            Utilization = utilization,
            PlanningResult = true,
        };
    }

    private DirectStackResult? FindBestLayerStack(
        IReadOnlyList<DirectLayerResult> candidates,
        IReadOnlyList<BoxDimension> dimensions)
    {
        var minimums = dimensions.Select(x => x.MinimumCount).ToArray();
        var keys = dimensions.Select(DimensionKey).ToArray();
        var memo = new Dictionary<string, DirectStackResult?>();
        return SearchLayerStack(candidates, StackMaxHeightMm, minimums, keys, memo);
    }

    private DirectStackResult? SearchLayerStack(
        IReadOnlyList<DirectLayerResult> candidates,
        double remainingHeight,
        IReadOnlyList<int> remainingMinimums,
        IReadOnlyList<string> dimensionKeys,
        IDictionary<string, DirectStackResult?> memo)
    {
        string key = string.Join("|",
            Math.Round(remainingHeight, 3, MidpointRounding.AwayFromZero)
                .ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            string.Join(",", remainingMinimums));
        if (memo.TryGetValue(key, out var cached)) return cached;

        bool requirementsMet = remainingMinimums.All(x => x <= 0);
        DirectStackResult? best = requirementsMet
            ? new DirectStackResult(Array.Empty<DirectLayerResult>(), 0, 0, string.Empty)
            : null;
        foreach (var candidate in candidates
                     .Where(x => x.HeightMm <= remainingHeight + Epsilon)
                     .OrderByDescending(x => x.OccupiedArea / x.HeightMm)
                     .ThenBy(x => x.Signature, StringComparer.Ordinal))
        {
            var nextMinimums = remainingMinimums.ToArray();
            foreach (var count in candidate.Placements
                         .GroupBy(x => DimensionKey(x.Dimension), StringComparer.Ordinal))
            {
                int index = Array.IndexOf(dimensionKeys.ToArray(), count.Key);
                if (index >= 0)
                    nextMinimums[index] = Math.Max(0, nextMinimums[index] - count.Count());
            }

            var tail = SearchLayerStack(
                candidates,
                remainingHeight - candidate.HeightMm,
                nextMinimums,
                dimensionKeys,
                memo);
            if (tail is null) continue;

            var plans = new[] { candidate }.Concat(tail.Layers).ToArray();
            var current = new DirectStackResult(
                plans,
                candidate.OccupiedArea + tail.TotalArea,
                candidate.Placements.Count + tail.TotalCount,
                string.Join("/", plans.Select(x => x.Signature)));
            if (best is null || IsBetterDirectStack(current, best)) best = current;
        }

        memo[key] = best;
        return best;
    }

    private static bool IsBetterDirectStack(DirectStackResult candidate, DirectStackResult current)
        => candidate.TotalArea > current.TotalArea + Epsilon
        || Math.Abs(candidate.TotalArea - current.TotalArea) <= Epsilon
        && (candidate.TotalCount > current.TotalCount
            || candidate.TotalCount == current.TotalCount
            && string.CompareOrdinal(candidate.Signature, current.Signature) < 0);

    private DirectLayerResult? FindBestDirectLayer(IReadOnlyList<BoxDimension> types)
    {
        if (types.Count == 0) return null;
        var frontier = new[] { new DirectLayerState(Array.Empty<DirectLayerPlacement>(), 0) };
        DirectLayerState? best = null;
        for (int depth = 0; depth < 256 && frontier.Length > 0; depth++)
        {
            var next = new List<DirectLayerState>();
            foreach (var state in frontier)
            foreach (var candidate in GenerateDirectCandidates(types, state.Placements))
            {
                if (state.Placements.Any(x => DirectCollides(candidate, x))) continue;
                next.Add(new DirectLayerState(
                    state.Placements.Concat(new[] { candidate }).ToArray(),
                    state.OccupiedArea + candidate.Width * candidate.Length));
            }

            frontier = next
                .OrderBy(x => MissingDirectTypeCount(x, types))
                .ThenByDescending(x => x.OccupiedArea)
                .ThenByDescending(x => x.Placements.Count)
                .ThenBy(x => DirectFragmentation(x))
                .ThenBy(DirectSignature, StringComparer.Ordinal)
                .Take(BeamWidth)
                .ToArray();
            var roundBest = frontier
                .Where(x => MissingDirectTypeCount(x, types) == 0)
                .OrderByDescending(x => x.OccupiedArea)
                .ThenByDescending(x => x.Placements.Count)
                .ThenBy(x => DirectFragmentation(x))
                .ThenBy(DirectSignature, StringComparer.Ordinal)
                .FirstOrDefault();
            if (roundBest is not null && (best is null || IsBetterDirectState(roundBest, best)))
                best = roundBest;
        }

        if (best is null || best.Placements.Count == 0) return null;
        return new DirectLayerResult(types[0].HeightMm, best.Placements, best.OccupiedArea, DirectSignature(best));
    }

    private static int MissingDirectTypeCount(
        DirectLayerState state,
        IReadOnlyList<BoxDimension> types)
        => types.Count(type => !state.Placements.Any(x =>
            string.Equals(DimensionKey(x.Dimension), DimensionKey(type), StringComparison.Ordinal)));

    private IEnumerable<DirectLayerPlacement> GenerateDirectCandidates(
        IReadOnlyList<BoxDimension> types,
        IReadOnlyList<DirectLayerPlacement> placed)
    {
        foreach (var dimension in types)
        foreach (var orientation in dimension.LengthMm == dimension.WidthMm ? new[] { 0 } : new[] { 0, 90 })
        {
            double width = orientation == 0 ? dimension.WidthMm : dimension.LengthMm;
            double length = orientation == 0 ? dimension.LengthMm : dimension.WidthMm;
            var xs = new SortedSet<double> { _palletXMinMm + width / 2 };
            var ys = new SortedSet<double> { _palletYMinMm + length / 2 };
            foreach (var item in placed)
            {
                xs.Add(item.Left - StackBoxGapMm - width / 2);
                xs.Add(item.Right + StackBoxGapMm + width / 2);
                ys.Add(item.Bottom - StackBoxGapMm - length / 2);
                ys.Add(item.Top + StackBoxGapMm + length / 2);
            }

            foreach (var x in xs)
            foreach (var y in ys)
            {
                var candidate = new DirectLayerPlacement(dimension, x, y, width, length, orientation);
                if (candidate.Left < _palletXMinMm - Epsilon || candidate.Right > _palletXMaxMm + Epsilon
                    || candidate.Bottom < _palletYMinMm - Epsilon || candidate.Top > _palletYMaxMm + Epsilon)
                    continue;
                if (placed.Any(item => DirectCollides(candidate, item))) continue;
                yield return candidate;
            }
        }
    }

    private static bool DirectCollides(DirectLayerPlacement a, DirectLayerPlacement b)
        => a.Left - StackBoxGapMm / 2 < b.Right + StackBoxGapMm / 2 - Epsilon
        && a.Right + StackBoxGapMm / 2 > b.Left - StackBoxGapMm / 2 + Epsilon
        && a.Bottom - StackBoxGapMm / 2 < b.Top + StackBoxGapMm / 2 - Epsilon
        && a.Top + StackBoxGapMm / 2 > b.Bottom - StackBoxGapMm / 2 + Epsilon;

    private static bool IsBetterDirectState(DirectLayerState candidate, DirectLayerState current)
        => candidate.OccupiedArea > current.OccupiedArea + Epsilon
        || Math.Abs(candidate.OccupiedArea - current.OccupiedArea) <= Epsilon
        && (candidate.Placements.Count > current.Placements.Count
            || candidate.Placements.Count == current.Placements.Count
            && string.CompareOrdinal(DirectSignature(candidate), DirectSignature(current)) < 0);

    private static double DirectFragmentation(DirectLayerState state)
    {
        if (state.Placements.Count == 0) return 0;
        double left = state.Placements.Min(x => x.Left);
        double right = state.Placements.Max(x => x.Right);
        double bottom = state.Placements.Min(x => x.Bottom);
        double top = state.Placements.Max(x => x.Top);
        return Math.Max(0, (right - left) * (top - bottom) - state.OccupiedArea);
    }

    private static string DirectSignature(DirectLayerState state) => string.Join(";", state.Placements
        .OrderBy(x => DimensionKey(x.Dimension), StringComparer.Ordinal)
        .ThenBy(x => x.Xmm).ThenBy(x => x.Ymm).ThenBy(x => x.OrientationDeg)
        .Select(x => string.Join("|", DimensionKey(x.Dimension), x.Xmm.ToString("R"), x.Ymm.ToString("R"), x.OrientationDeg)));

    private static UtilizationPlanResult EmptyUtilizationPlan(
        string message,
        bool planningResult = false) => new()
    {
        PlannedCounts = Array.Empty<BoxTypeCount>(),
        Layers = Array.Empty<LayerUtilizationSummary>(),
        Utilization = 0,
        PlanningResult = planningResult,
        Message = message,
    };


    /// <summary>
    /// 使用当前真实箱子集合执行有限数量的二维利用率规划，并直接返回逐箱堆垛数据。
    /// 本方法复用 <see cref="PlanBestUtilization(IEnumerable{BoxDimension})" /> 的
    /// 同高度分层、最多三种箱型组合和二维候选搜索思路，但不会调用正式的
    /// <see cref="GeneratePlan()" />，也不会重新生成箱子数量或箱号。
    /// </summary>
    /// <remarks>
    /// 利用率规划面向“当前仍在货架上的箱子集合”。调用方应在规划前完成箱子的增删；
    /// 每个真实箱子最多使用一次，输出中的箱号和 Order 与当前集合保持对应。
    /// </remarks>
    public StackPlanResult GenerateBestUtilizationPlan()
    {
        ApplyPendingUtilizationOrders();
        AssignOrders(preserveExistingOnShelf: true);
        var boxes = _group.MutableBoxes
            .Where(x => x.Status == BoxStatus.OnShelf)
            .OrderBy(x => x.Order)
            .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
            .ToArray();

        if (_group.MutableBoxes.Any(x => x.Status != BoxStatus.OnShelf))
        {
            _lastPlan = new StackPlanResult
            {
                Placements = Array.Empty<BoxPlacement>(),
                PlanningResult = false,
            };
            _hasGeneratedPlan = true;
            return ClonePlan(_lastPlan);
        }

        var remaining = boxes.ToList();
        var placements = new List<BoxPlacement>();
        double baseZ = 0;
        int layerIndex = 0;
        while (remaining.Count > 0)
        {
            var layerCandidates = new List<DirectBoxLayerResult>();
            foreach (var heightGroup in remaining.GroupBy(x => x.HeightMm).OrderBy(x => x.Key))
            {
                if (baseZ + heightGroup.Key > StackMaxHeightMm + Epsilon)
                    continue;

                var typeKeys = heightGroup
                    .Select(BoxTypeKey)
                    .Distinct(StringComparer.Ordinal)
                    .OrderBy(x => x, StringComparer.Ordinal)
                    .ToArray();
                foreach (var combination in TypeCombinations(typeKeys, 3))
                {
                    var candidate = FindBestFiniteDirectLayer(
                        remaining.Where(x => combination.Contains(BoxTypeKey(x), StringComparer.Ordinal)).ToArray(),
                        heightGroup.Key,
                        combination.ToHashSet(StringComparer.Ordinal));
                    if (candidate is not null)
                        layerCandidates.Add(candidate);
                }
            }

            var selected = layerCandidates
                .OrderByDescending(x => x.OccupiedArea)
                .ThenByDescending(x => x.Placements.Count)
                .ThenBy(x => DirectBoxFragmentation(x.Placements))
                .ThenBy(x => x.HeightMm)
                .ThenBy(x => x.Signature, StringComparer.Ordinal)
                .FirstOrDefault();
            if (selected is null)
                break;

            foreach (var item in selected.Placements)
            {
                placements.Add(new BoxPlacement
                {
                    Order = item.Box.Order,
                    BoxNumber = item.Box.BoxNumber,
                    Xmm = Round(item.Xmm),
                    Ymm = Round(item.Ymm),
                    Zmm = Round(PlaceFloorZMm - baseZ - item.Box.HeightMm),
                    OrientationDeg = item.OrientationDeg,
                    LayerIndex = layerIndex,
                });
            }

            var usedNumbers = selected.Placements
                .Select(x => x.Box.BoxNumber)
                .ToHashSet(StringComparer.Ordinal);
            remaining.RemoveAll(x => usedNumbers.Contains(x.BoxNumber));
            baseZ += selected.HeightMm;
            layerIndex++;
        }

        var ordered = placements
            .OrderBy(x => x.LayerIndex)
            .ThenByDescending(x => x.Xmm)
            .ThenByDescending(x => x.Ymm)
            .ThenBy(x => x.Order)
            .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
            .ToArray();
        ordered = NormalizePlacementOrders(ordered);
        foreach (var placement in ordered)
            FindBox(placement.BoxNumber)!.Order = placement.Order;
        _lastPlacements.Clear();
        foreach (var placement in ordered)
            _lastPlacements[placement.BoxNumber] = ClonePlacement(placement);
        _lastPlan = new StackPlanResult
        {
            Placements = ordered,
            PlanningResult = ordered.Length == boxes.Length,
        };
        _hasGeneratedPlan = true;
        return ClonePlan(_lastPlan);
    }

    private DirectBoxLayerResult? FindBestFiniteDirectLayer(
        IReadOnlyList<Box> boxes,
        double heightMm,
        IReadOnlySet<string> allowedTypeKeys)
    {
        if (boxes.Count == 0)
            return null;

        var frontier = new[] { new DirectBoxLayerState(Array.Empty<DirectBoxLayerPlacement>(), 0, boxes) };
        DirectBoxLayerState? best = null;
        for (int depth = 0; depth < boxes.Count && frontier.Length > 0; depth++)
        {
            var next = new List<DirectBoxLayerState>();
            foreach (var state in frontier)
            {
                var representatives = state.Remaining
                    .Where(x => allowedTypeKeys.Contains(BoxTypeKey(x)))
                    .GroupBy(BoxTypeKey, StringComparer.Ordinal)
                    .Select(group => group.OrderBy(x => x.Order).ThenBy(x => x.BoxNumber, StringComparer.Ordinal).First())
                    .OrderBy(x => BoxTypeKey(x), StringComparer.Ordinal)
                    .ToArray();
                foreach (var candidate in GenerateFiniteDirectCandidates(representatives, state.Placements))
                {
                    if (state.Placements.Any(x => DirectBoxCollides(candidate, x)))
                        continue;
                    var remaining = state.Remaining
                        .Where(x => !string.Equals(x.BoxNumber, candidate.Box.BoxNumber, StringComparison.Ordinal))
                        .ToArray();
                    next.Add(new DirectBoxLayerState(
                        state.Placements.Concat(new[] { candidate }).ToArray(),
                        state.OccupiedArea + candidate.Width * candidate.Length,
                        remaining));
                }
            }

            frontier = next
                .OrderBy(x => MissingDirectBoxTypeCount(x, allowedTypeKeys))
                .ThenByDescending(x => x.OccupiedArea)
                .ThenByDescending(x => x.Placements.Count)
                .ThenBy(x => DirectBoxFragmentation(x.Placements))
                .ThenBy(DirectBoxSignature, StringComparer.Ordinal)
                .Take(BeamWidth)
                .ToArray();
            var roundBest = frontier
                .Where(x => MissingDirectBoxTypeCount(x, allowedTypeKeys) == 0)
                .OrderByDescending(x => x.OccupiedArea)
                .ThenByDescending(x => x.Placements.Count)
                .ThenBy(x => DirectBoxFragmentation(x.Placements))
                .ThenBy(DirectBoxSignature, StringComparer.Ordinal)
                .FirstOrDefault();
            if (roundBest is not null && (best is null || IsBetterFiniteDirectState(roundBest, best)))
                best = roundBest;
        }

        return best is null || best.Placements.Count == 0
            ? null
            : new DirectBoxLayerResult(heightMm, best.Placements, best.OccupiedArea, DirectBoxSignature(best));
    }

    private IEnumerable<DirectBoxLayerPlacement> GenerateFiniteDirectCandidates(
        IReadOnlyList<Box> boxes,
        IReadOnlyList<DirectBoxLayerPlacement> placed)
    {
        foreach (var box in boxes)
        foreach (var orientation in box.LengthMm == box.WidthMm ? new[] { 0 } : new[] { 0, 90 })
        {
            double width = orientation == 0 ? box.WidthMm : box.LengthMm;
            double length = orientation == 0 ? box.LengthMm : box.WidthMm;
            var xs = new SortedSet<double> { _palletXMinMm + width / 2 };
            var ys = new SortedSet<double> { _palletYMinMm + length / 2 };
            foreach (var item in placed)
            {
                xs.Add(item.Left - StackBoxGapMm - width / 2);
                xs.Add(item.Right + StackBoxGapMm + width / 2);
                ys.Add(item.Bottom - StackBoxGapMm - length / 2);
                ys.Add(item.Top + StackBoxGapMm + length / 2);
            }

            foreach (var x in xs)
            foreach (var y in ys)
            {
                var candidate = new DirectBoxLayerPlacement(box, x, y, width, length, orientation);
                if (candidate.Left < _palletXMinMm - Epsilon || candidate.Right > _palletXMaxMm + Epsilon
                    || candidate.Bottom < _palletYMinMm - Epsilon || candidate.Top > _palletYMaxMm + Epsilon)
                    continue;
                if (placed.Any(item => DirectBoxCollides(candidate, item)))
                    continue;
                yield return candidate;
            }
        }
    }

    private static int MissingDirectBoxTypeCount(
        DirectBoxLayerState state,
        IReadOnlySet<string> allowedTypeKeys)
        => allowedTypeKeys.Count(type => !state.Placements.Any(x => BoxTypeKey(x.Box) == type));

    private static bool IsBetterFiniteDirectState(DirectBoxLayerState candidate, DirectBoxLayerState current)
        => candidate.OccupiedArea > current.OccupiedArea + Epsilon
        || Math.Abs(candidate.OccupiedArea - current.OccupiedArea) <= Epsilon
        && (candidate.Placements.Count > current.Placements.Count
            || candidate.Placements.Count == current.Placements.Count
            && string.CompareOrdinal(DirectBoxSignature(candidate), DirectBoxSignature(current)) < 0);

    private static double DirectBoxFragmentation(IReadOnlyList<DirectBoxLayerPlacement> placements)
    {
        if (placements.Count == 0) return 0;
        double left = placements.Min(x => x.Left);
        double right = placements.Max(x => x.Right);
        double bottom = placements.Min(x => x.Bottom);
        double top = placements.Max(x => x.Top);
        double area = placements.Sum(x => x.Width * x.Length);
        return Math.Max(0, (right - left) * (top - bottom) - area);
    }

    private static string DirectBoxSignature(DirectBoxLayerState state)
        => string.Join(";", state.Placements
            .OrderBy(x => BoxTypeKey(x.Box), StringComparer.Ordinal)
            .ThenBy(x => x.Xmm)
            .ThenBy(x => x.Ymm)
            .ThenBy(x => x.OrientationDeg)
            .ThenBy(x => x.Box.BoxNumber, StringComparer.Ordinal)
            .Select(x => string.Join("|", BoxTypeKey(x.Box),
                x.Xmm.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
                x.Ymm.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
                x.OrientationDeg, x.Box.BoxNumber)));

    private static bool DirectBoxCollides(DirectBoxLayerPlacement a, DirectBoxLayerPlacement b)
        => a.Left - StackBoxGapMm / 2 < b.Right + StackBoxGapMm / 2 - Epsilon
        && a.Right + StackBoxGapMm / 2 > b.Left - StackBoxGapMm / 2 + Epsilon
        && a.Bottom - StackBoxGapMm / 2 < b.Top + StackBoxGapMm / 2 - Epsilon
        && a.Top + StackBoxGapMm / 2 > b.Bottom - StackBoxGapMm / 2 + Epsilon;
    private double CalculateLayerUtilization(int layer)
    {
        double palletArea = (_palletXMaxMm - _palletXMinMm) * (_palletYMaxMm - _palletYMinMm);
        if (palletArea <= Epsilon) return 0;
        return _lastPlan.Placements
            .Where(x => x.LayerIndex == layer)
            .Select(x => ToPlacedBox(FindBox(x.BoxNumber)!, x, false))
            .Sum(x => x.Width * x.Length) / palletArea;
    }

    private static BoxDimension ToDimension(Box box) => new()
    {
        LengthMm = box.LengthMm,
        WidthMm = box.WidthMm,
        HeightMm = box.HeightMm,
    };

    private static string DimensionKey(BoxDimension dimension)
        => string.Join("x", Math.Min(dimension.LengthMm, dimension.WidthMm).ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            Math.Max(dimension.LengthMm, dimension.WidthMm).ToString("R", System.Globalization.CultureInfo.InvariantCulture),
            dimension.HeightMm.ToString("R", System.Globalization.CultureInfo.InvariantCulture));

    private StackPlanner CreatePlanningCopy()
    {
        var copy = new StackPlanner(_palletXMinMm, _palletXMaxMm, _palletYMinMm, _palletYMaxMm);
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

    /// <summary>记录新增箱子是否匹配待补位的同规格删除箱。</summary>
    private void RegisterReplacementBox(Box box)
    {
        string typeKey = BoxTypeKey(box);
        if (!_pendingReplacementCounts.TryGetValue(typeKey, out int pendingCount) || pendingCount <= 0)
            return;

        if (pendingCount == 1)
            _pendingReplacementCounts.Remove(typeKey);
        else
            _pendingReplacementCounts[typeKey] = pendingCount - 1;

        if (!_pendingReplacementBoxes.TryGetValue(typeKey, out var boxNumbers))
        {
            boxNumbers = new HashSet<string>(StringComparer.Ordinal);
            _pendingReplacementBoxes[typeKey] = boxNumbers;
        }
        boxNumbers.Add(box.BoxNumber);
    }

    /// <summary>
    /// 仅为利用率规划应用同规格替换箱的末尾序号。
    /// 不修改 GeneratePlan 使用的增删箱子顺序逻辑。
    /// </summary>
    private void ApplyPendingUtilizationOrders()
    {
        foreach (var entry in _pendingReplacementBoxes.ToArray())
        {
            string typeKey = entry.Key;
            var replacementBoxes = _group.MutableBoxes
                .Where(x => entry.Value.Contains(x.BoxNumber)
                    && x.Status is not (BoxStatus.Stacking or BoxStatus.StackingSucceeded))
                .ToArray();
            if (replacementBoxes.Length == 0)
                continue;

            var eligible = _group.MutableBoxes
                .Where(x => x.Status is not (BoxStatus.Stacking or BoxStatus.StackingSucceeded)
                    && string.Equals(BoxTypeKey(x), typeKey, StringComparison.Ordinal)
                    && !entry.Value.Contains(x.BoxNumber)
                    && x.Order >= 0)
                .ToArray();
            int nextOrder = eligible.Select(x => x.Order).DefaultIfEmpty(-1).Max() + 1;
            var usedOrders = _group.MutableBoxes.Where(x => x.Order >= 0).Select(x => x.Order).ToHashSet();
            foreach (var box in replacementBoxes.OrderBy(x => x.BoxNumber, StringComparer.Ordinal))
            {
                while (usedOrders.Contains(nextOrder)) nextOrder++;
                box.Order = nextOrder++;
                usedOrders.Add(box.Order);
            }

            _pendingReplacementBoxes.Remove(typeKey);
        }
    }

    private void RemoveUtilizationBox(string boxNumber)
    {
        var removed = FindBox(boxNumber)!;
        string removedKey = BoxTypeKey(removed);
        var oldPlacements = _lastPlan.Placements.ToArray();
        var groups = oldPlacements
            .Where(point => FindBox(point.BoxNumber) is not null)
            .GroupBy(point => BoxTypeKey(FindBox(point.BoxNumber)!), StringComparer.Ordinal)
            .ToArray();
        var replacements = new Dictionary<string, BoxPlacement>(StringComparer.Ordinal);

        foreach (var group in groups)
        {
            var points = group.ToArray();
            var survivors = points
                .Where(point => !string.Equals(point.BoxNumber, boxNumber, StringComparison.Ordinal))
                .ToArray();
            var sourcePoints = points
                .Where(point => !string.Equals(point.BoxNumber, boxNumber, StringComparison.Ordinal))
                .ToArray();
            if (string.Equals(group.Key, removedKey, StringComparison.Ordinal))
            {
                sourcePoints = points;
                for (int i = 0; i < survivors.Length; i++)
                    replacements[survivors[i].BoxNumber] = sourcePoints[i] with
                    {
                        BoxNumber = survivors[i].BoxNumber,
                    };
                if (points.Length > survivors.Length)
                {
                    var tail = sourcePoints[^1];
                    _reusableUtilizationPlacements
                        .GetValueOrDefault(removedKey, new List<BoxPlacement>())
                        .Add(tail);
                    if (!_reusableUtilizationPlacements.ContainsKey(removedKey))
                        _reusableUtilizationPlacements[removedKey] = new List<BoxPlacement> { tail };
                }
            }
        }

        _group.MutableBoxes.Remove(removed);
        _lastPlacements.Clear();
        foreach (var point in oldPlacements)
        {
            if (string.Equals(point.BoxNumber, boxNumber, StringComparison.Ordinal))
                continue;
            var replacement = replacements.GetValueOrDefault(point.BoxNumber, point);
            _lastPlacements[replacement.BoxNumber] = replacement;
        }
        _lastPlan = new StackPlanResult
        {
            Placements = oldPlacements
                .Where(point => !string.Equals(point.BoxNumber, boxNumber, StringComparison.Ordinal))
                .Select(point => replacements.GetValueOrDefault(point.BoxNumber, point))
                .ToArray(),
            PlanningResult = true,
        };
    }

    private static BoxPlacement[] NormalizePlacementOrders(IReadOnlyList<BoxPlacement> placements)
        => placements
            .Select((placement, index) => placement with { Order = index })
            .ToArray();

    private UtilizationMutationResult MutationSuccess(string message)
        => new()
        {
            Succeeded = true,
            Message = message,
            Boxes = GetBoxes(),
            Plan = CurrentPlan,
            ReusablePlacements = ExportReusablePlacements(),
        };

    private UtilizationMutationResult MutationFailure(string message)
        => new()
        {
            Succeeded = false,
            Message = message,
            Boxes = GetBoxes(),
            Plan = CurrentPlan,
            ReusablePlacements = ExportReusablePlacements(),
        };

    private IReadOnlyList<ReusablePlacementSet> ExportReusablePlacements()
        => _reusableUtilizationPlacements
            .Where(entry => entry.Value.Count > 0)
            .Select(entry => new ReusablePlacementSet
            {
                Dimension = ParseDimensionKey(entry.Key),
                Placements = entry.Value.Select(ClonePlacement).ToArray(),
            })
            .ToArray();

    private static BoxDimension ParseDimensionKey(string key)
    {
        var values = key.Split('x');
        return new BoxDimension
        {
            LengthMm = double.Parse(values[0], System.Globalization.CultureInfo.InvariantCulture),
            WidthMm = double.Parse(values[1], System.Globalization.CultureInfo.InvariantCulture),
            HeightMm = double.Parse(values[2], System.Globalization.CultureInfo.InvariantCulture),
        };
    }

    private void AssignOrders(bool preserveExistingOnShelf = false)
    {
        // 非 OnShelf 箱子的 Order 是业务流程已经锁定的顺序，不能重新编号。
        // OnShelf 箱子严格按照加入集合的顺序分配顺序号，不再按箱型或箱号排序。
        var boxes = _group.MutableBoxes;
        if (boxes.Count > 0 && boxes.All(x => x.Status == BoxStatus.OnShelf))
        {
            if (!preserveExistingOnShelf)
            {
                int order = 0;
                foreach (var box in boxes)
                    box.Order = order++;
                return;
            }

            int nextOrder = boxes
                .Where(x => x.Order >= 0)
                .Select(x => x.Order)
                .DefaultIfEmpty(-1)
                .Max() + 1;
            foreach (var box in boxes.Where(x => x.Order < 0))
                box.Order = nextOrder++;
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
            var xValues = new SortedSet<double> { _palletXMinMm + EdgeMarginMm + width / 2 };
            var yValues = new SortedSet<double> { _palletYMinMm + EdgeMarginMm + length / 2 };
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
    private bool Fits(PlacedBox item, IReadOnlyList<PlacedBox> occupied) => InsidePallet(item) && !Collides(item, occupied);
    private bool InsidePallet(Candidate x) => x.X - x.Width / 2 >= _palletXMinMm - Epsilon && x.X + x.Width / 2 <= _palletXMaxMm + Epsilon
        && x.Y - x.Length / 2 >= _palletYMinMm - Epsilon && x.Y + x.Length / 2 <= _palletYMaxMm + Epsilon
        && x.BaseZ >= -Epsilon && x.BaseZ + x.Height <= StackMaxHeightMm + Epsilon;
    private bool InsidePallet(PlacedBox x) => x.Left >= _palletXMinMm - Epsilon && x.Right <= _palletXMaxMm + Epsilon
        && x.Bottom >= _palletYMinMm - Epsilon && x.Top <= _palletYMaxMm + Epsilon && x.BaseZ >= -Epsilon
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
    /// 对仍在货架上的箱子执行确定性的分层搜索。
    /// 中间层只提交达到最低利用率的混合布局；剩余箱子统一作为最高层处理。
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

        var current = initial;
        while (current.Remaining.Count > 0)
        {
            var layerCandidate = FindBestSingleLayerCandidate(current);
            if (layerCandidate is null)
                break;

            var layer = layerCandidate.State;
            int layerIndex = layerCandidate.LayerIndex;
            current = layer;
        }

        // 没有任何可放位置时结束；尾部模式已经在上面的分支中完成。
        if (current.Remaining.Count > 0)
        {
            var topLayer = FindBestSingleLayerCandidate(current)?.State;
            if (topLayer is not null)
                current = topLayer;
        }

        return current;
    }

    /// <summary>
    /// 在当前最低可用层执行一次混合箱型有限宽度单层装箱。
    /// 布局比较严格按利用率、碎片、包围区域、支撑面积和确定性签名进行。
    /// </summary>
    private LayerSearchCandidate? FindBestSingleLayerCandidate(SearchState initial)
    {
        var heightGroups = new List<List<Box>>();
        foreach (var box in initial.Remaining
                     .OrderBy(x => x.HeightMm)
                     .ThenBy(x => x.Order)
                     .ThenBy(x => x.BoxNumber, StringComparer.Ordinal))
        {
            var group = heightGroups.LastOrDefault();
            if (group is null || Math.Abs(group[0].HeightMm - box.HeightMm) > Epsilon)
            {
                heightGroups.Add(new List<Box> { box });
            }
            else
            {
                group.Add(box);
            }
        }

        LayerSearchCandidate? best = null;
        foreach (var group in heightGroups)
        {
            double height = group[0].HeightMm;
            var typeKeys = group
                .Select(BoxTypeKey)
                .Distinct(StringComparer.Ordinal)
                .OrderBy(x => x, StringComparer.Ordinal)
                .ToArray();
            foreach (var combination in TypeCombinations(typeKeys, maxSize: 3))
            {
                var layer = TryPackBestSingleLayer(
                    initial, height, combination.ToHashSet(StringComparer.Ordinal));
                AddLayerCandidate(ref best, initial, layer, singleType: combination.Length == 1, height);
            }
        }

        return best;
    }

    private void AddLayerCandidate(
        ref LayerSearchCandidate? best,
        SearchState initial,
        SearchState? state,
        bool singleType,
        double height)
    {
        if (state is null)
            return;

        int layer = GetSearchLayer(state, initial);
        if (layer == int.MaxValue)
            return;

        var candidate = new LayerSearchCandidate(state, singleType, height, layer);
        if (best is null || IsBetterLayerCandidate(candidate, best))
            best = candidate;
    }

    private bool IsBetterLayerCandidate(
        LayerSearchCandidate candidate,
        LayerSearchCandidate current)
    {
        double candidateUtilization = SearchLayerUtilization(candidate.State, candidate.LayerIndex);
        double currentUtilization = SearchLayerUtilization(current.State, current.LayerIndex);
        if (candidateUtilization > currentUtilization + Epsilon) return true;
        if (currentUtilization > candidateUtilization + Epsilon) return false;

        if (candidate.SingleType != current.SingleType)
            return candidate.SingleType;

        return IsBetterSingleLayer(candidate.State, current.State, candidate.LayerIndex);
    }

    private SearchState? TryPackBestSingleLayer(
        SearchState initial,
        double? requiredHeight = null,
        IReadOnlySet<string>? allowedTypeKeys = null,
        bool startAboveHighest = true)
    {
        var eligible = initial.Remaining
            .Where(x => (!requiredHeight.HasValue
                || Math.Abs(x.HeightMm - requiredHeight.Value) <= Epsilon)
                && (allowedTypeKeys is null || allowedTypeKeys.Contains(BoxTypeKey(x))))
            .ToArray();
        if (eligible.Length == 0)
            return null;

        int highestOccupiedLayer = initial.Occupied
            .Select(x => x.Placement.LayerIndex)
            .DefaultIfEmpty(-1)
            .Max();
        int targetLayer = startAboveHighest && highestOccupiedLayer >= 0
            ? highestOccupiedLayer + 1
            : eligible
                .SelectMany(box => GenerateCandidates(box, initial.Occupied))
                .Select(candidate => candidate.Placement.LayerIndex)
                .DefaultIfEmpty(int.MaxValue)
                .Min();
        if (targetLayer == int.MaxValue)
            return null;

        var frontier = new[] { initial };
        SearchState best = initial;
        for (int depth = 0; depth < eligible.Length && frontier.Length > 0; depth++)
        {
            var next = new List<SearchState>();
            foreach (var state in frontier)
            {
                bool expanded = false;
                // 几何完全相同的箱子只扩展确定性的代表箱，避免为同型箱子的交换顺序
                // 重复搜索；提交结果时仍然按 Order 和箱号绑定到具体箱子。
                foreach (var box in state.Remaining
                             .Where(x => (!requiredHeight.HasValue
                                 || Math.Abs(x.HeightMm - requiredHeight.Value) <= Epsilon)
                                 && (allowedTypeKeys is null || allowedTypeKeys.Contains(BoxTypeKey(x))))
                             .GroupBy(BoxTypeKey, StringComparer.Ordinal)
                             .Select(group => group
                                 .OrderBy(x => x.Order)
                                 .ThenBy(x => x.BoxNumber, StringComparer.Ordinal)
                                 .First())
                             .OrderByDescending(BoxBaseArea)
                             .ThenByDescending(BoxVolume)
                             .ThenBy(x => x.Order)
                             .ThenBy(x => x.BoxNumber, StringComparer.Ordinal))
                {
                    foreach (var candidate in GenerateCandidates(box, state.Occupied)
                                 .Where(x => x.Placement.LayerIndex == targetLayer))
                    {
                        var placed = ToPlacedBox(box, candidate.Placement, false);
                        if (!Fits(placed, state.Occupied)) continue;

                        var placements = new Dictionary<string, BoxPlacement>(
                            state.Placements, StringComparer.Ordinal)
                        {
                            [box.BoxNumber] = candidate.Placement,
                        };
                        var remaining = state.Remaining
                            .Where(x => !ReferenceEquals(x, box)).ToArray();
                        next.Add(new SearchState(
                            remaining,
                            state.Occupied.Concat(new[] { placed }).ToArray(),
                            placements,
                            targetLayer,
                            BoxTypeKey(box)));
                        expanded = true;
                    }
                }

                // 保留当前层无法继续扩展的终止状态，供评分选择。
                if (!expanded)
                    next.Add(state);
            }

            frontier = next
                .OrderByDescending(x => SearchLayerUtilization(x, targetLayer))
                .ThenBy(x => SearchLayerFragmentation(x, targetLayer))
                .ThenBy(x => SearchLayerBoundingArea(x, targetLayer))
                .ThenByDescending(x => SearchLayerSupportArea(x, targetLayer))
                .ThenBy(SearchSignature, StringComparer.Ordinal)
                .Take(BeamWidth)
                .ToArray();

            var roundBest = frontier
                .OrderByDescending(x => SearchLayerUtilization(x, targetLayer))
                .ThenBy(x => SearchLayerFragmentation(x, targetLayer))
                .ThenBy(x => SearchLayerBoundingArea(x, targetLayer))
                .ThenByDescending(x => SearchLayerSupportArea(x, targetLayer))
                .ThenBy(SearchSignature, StringComparer.Ordinal)
                .FirstOrDefault();
            if (roundBest is not null && IsBetterSingleLayer(roundBest, best, targetLayer))
                best = roundBest;
        }

        return best.Placements.Count > initial.Placements.Count ? best : null;
    }

    private static IEnumerable<string[]> TypeCombinations(IReadOnlyList<string> keys, int maxSize)
    {
        int limit = Math.Min(maxSize, keys.Count);
        for (int size = 1; size <= limit; size++)
        {
            foreach (var combination in BuildTypeCombinations(keys, size, 0, new List<string>()))
                yield return combination;
        }
    }

    private static IEnumerable<string[]> BuildTypeCombinations(
        IReadOnlyList<string> keys, int size, int start, List<string> current)
    {
        if (current.Count == size)
        {
            yield return current.ToArray();
            yield break;
        }

        for (int index = start; index <= keys.Count - (size - current.Count); index++)
        {
            current.Add(keys[index]);
            foreach (var combination in BuildTypeCombinations(keys, size, index + 1, current))
                yield return combination;
            current.RemoveAt(current.Count - 1);
        }
    }

    private static IEnumerable<BoxDimension[]> TypeCombinations(
        IReadOnlyList<BoxDimension> dimensions, int maxSize)
    {
        int limit = Math.Min(maxSize, dimensions.Count);
        for (int size = 1; size <= limit; size++)
        {
            foreach (var combination in BuildDimensionCombinations(dimensions, size, 0, new List<BoxDimension>()))
                yield return combination;
        }
    }

    private static IEnumerable<BoxDimension[]> BuildDimensionCombinations(
        IReadOnlyList<BoxDimension> dimensions, int size, int start, List<BoxDimension> current)
    {
        if (current.Count == size)
        {
            yield return current.ToArray();
            yield break;
        }

        for (int index = start; index <= dimensions.Count - (size - current.Count); index++)
        {
            current.Add(dimensions[index]);
            foreach (var combination in BuildDimensionCombinations(dimensions, size, index + 1, current))
                yield return combination;
            current.RemoveAt(current.Count - 1);
        }
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

    private int GetSearchLayer(SearchState state, SearchState previous)
    {
        var addedNumbers = state.Placements.Keys
            .Where(number => !previous.Placements.ContainsKey(number))
            .ToHashSet(StringComparer.Ordinal);
        return state.Placements.Values
            .Where(x => addedNumbers.Contains(x.BoxNumber))
            .Select(x => x.LayerIndex)
            .DefaultIfEmpty(int.MaxValue)
            .Min();
    }

    private double SearchLayerUtilization(SearchState state, int layer)
    {
        if (layer == int.MaxValue)
            return 0;

        double palletArea = (_palletXMaxMm - _palletXMinMm)
            * (_palletYMaxMm - _palletYMinMm);
        if (palletArea <= Epsilon)
            return 0;

        double area = state.Occupied
            .Where(x => x.Placement.LayerIndex == layer)
            .Sum(x => x.Width * x.Length);
        return area / palletArea;
    }

    private static double SearchLayerBoundingArea(SearchState state, int layer)
    {
        var boxes = state.Occupied.Where(x => x.Placement.LayerIndex == layer).ToArray();
        if (boxes.Length == 0)
            return 0;

        return (boxes.Max(x => x.Right) - boxes.Min(x => x.Left))
            * (boxes.Max(x => x.Top) - boxes.Min(x => x.Bottom));
    }

    private static double SearchLayerFragmentation(SearchState state, int layer)
    {
        var boxes = state.Occupied.Where(x => x.Placement.LayerIndex == layer).ToArray();
        if (boxes.Length == 0)
            return double.PositiveInfinity;

        double occupiedArea = boxes.Sum(x => x.Width * x.Length);
        return Math.Max(0, SearchLayerBoundingArea(state, layer) - occupiedArea);
    }

    private static double SearchLayerSupportArea(SearchState state, int layer)
    {
        double supportArea = 0;
        foreach (var item in state.Occupied.Where(x => x.Placement.LayerIndex == layer))
        {
            supportArea += state.Occupied
                .Where(x => x.TopZ <= item.BaseZ + Epsilon
                    && Math.Abs(x.TopZ - item.BaseZ) <= Epsilon)
                .Sum(x => OverlapLength(item.Left, item.Right, x.Left, x.Right)
                    * OverlapLength(item.Bottom, item.Top, x.Bottom, x.Top));
        }
        return supportArea;
    }

    private bool IsBetterSingleLayer(SearchState candidate, SearchState current, int layer)
    {
        double candidateUtilization = SearchLayerUtilization(candidate, layer);
        double currentUtilization = SearchLayerUtilization(current, layer);
        if (candidateUtilization > currentUtilization + Epsilon) return true;
        if (currentUtilization > candidateUtilization + Epsilon) return false;

        double candidateFragmentation = SearchLayerFragmentation(candidate, layer);
        double currentFragmentation = SearchLayerFragmentation(current, layer);
        if (candidateFragmentation < currentFragmentation - Epsilon) return true;
        if (currentFragmentation < candidateFragmentation - Epsilon) return false;

        double candidateBounding = SearchLayerBoundingArea(candidate, layer);
        double currentBounding = SearchLayerBoundingArea(current, layer);
        if (candidateBounding < currentBounding - Epsilon) return true;
        if (currentBounding < candidateBounding - Epsilon) return false;

        double candidateSupport = SearchLayerSupportArea(candidate, layer);
        double currentSupport = SearchLayerSupportArea(current, layer);
        if (candidateSupport > currentSupport + Epsilon) return true;
        if (currentSupport > candidateSupport + Epsilon) return false;

        return string.CompareOrdinal(SearchSignature(candidate), SearchSignature(current)) < 0;
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
    private sealed record LayerSearchCandidate(
        SearchState State,
        bool SingleType,
        double HeightMm,
        int LayerIndex);
    /// <summary>同箱型一层布局的相对槽位。</summary>
    private sealed record LayerTemplateSlot(double RelativeX, double RelativeY, int OrientationDeg);
    /// <summary>同箱型一层布局模板，不包含具体箱号和绝对高度。</summary>
    private sealed record LayerTemplate(IReadOnlyList<LayerTemplateSlot> Slots);
    /// <summary>直接二维单层搜索中的一个箱子放置结果。</summary>
    private sealed record DirectLayerPlacement(
        BoxDimension Dimension,
        double Xmm,
        double Ymm,
        double Width,
        double Length,
        int OrientationDeg)
    {
        public double Left => Xmm - Width / 2;
        public double Right => Xmm + Width / 2;
        public double Bottom => Ymm - Length / 2;
        public double Top => Ymm + Length / 2;
    }
    /// <summary>直接二维单层搜索状态。</summary>
    private sealed record DirectLayerState(
        IReadOnlyList<DirectLayerPlacement> Placements,
        double OccupiedArea);
    /// <summary>一个确定性的单层布局候选。</summary>
    private sealed record DirectLayerResult(
        double HeightMm,
        IReadOnlyList<DirectLayerPlacement> Placements,
        double OccupiedArea,
        string Signature);
    /// <summary>有限真实箱子的一层二维放置结果。</summary>
    private sealed record DirectBoxLayerPlacement(
        Box Box,
        double Xmm,
        double Ymm,
        double Width,
        double Length,
        int OrientationDeg)
    {
        public double Left => Xmm - Width / 2;
        public double Right => Xmm + Width / 2;
        public double Bottom => Ymm - Length / 2;
        public double Top => Ymm + Length / 2;
    }
    /// <summary>有限真实箱子的一层二维搜索状态。</summary>
    private sealed record DirectBoxLayerState(
        IReadOnlyList<DirectBoxLayerPlacement> Placements,
        double OccupiedArea,
        IReadOnlyList<Box> Remaining);
    /// <summary>绑定真实箱子的单层二维布局候选。</summary>
    private sealed record DirectBoxLayerResult(
        double HeightMm,
        IReadOnlyList<DirectBoxLayerPlacement> Placements,
        double OccupiedArea,
        string Signature);
    /// <summary>不同高度单层方案组合后的整垛结果。</summary>
    private sealed record DirectStackResult(
        IReadOnlyList<DirectLayerResult> Layers,
        double TotalArea,
        int TotalCount,
        string Signature);
}
