using System.Text.Json;
using System.Text.Json.Serialization;

namespace StackPlanner.Core;

/// <summary>负责规划快照的原子文件读写。</summary>
public static class PlannerSnapshotStore
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true,
        Converters = { new JsonStringEnumConverter() },
    };

    /// <summary>读取并校验规划快照。</summary>
    public static PlannerSnapshot Load(string filePath)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(filePath);
        using var stream = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
        var snapshot = JsonSerializer.Deserialize<PlannerSnapshot>(stream, JsonOptions)
            ?? throw new InvalidDataException($"无法解析规划快照：{filePath}");
        if (snapshot.SchemaVersion != 1)
            throw new InvalidDataException($"不支持的规划快照版本：{snapshot.SchemaVersion}");
        return snapshot;
    }

    /// <summary>
    /// 以临时文件加同目录替换的方式发布快照，返回实际写入的递增版本号。
    /// </summary>
    public static PlannerSnapshot Save(string filePath, PlannerSnapshot snapshot)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(filePath);
        ArgumentNullException.ThrowIfNull(snapshot);

        string fullPath = Path.GetFullPath(filePath);
        string? directory = Path.GetDirectoryName(fullPath);
        if (!string.IsNullOrEmpty(directory))
            Directory.CreateDirectory(directory);

        long previousRevision = 0;
        if (File.Exists(fullPath))
        {
            try
            {
                previousRevision = Load(fullPath).Revision;
            }
            catch (IOException)
            {
                // 正式文件将在本次完整写入成功后替换，读取竞争不影响发布。
            }
            catch (JsonException)
            {
                // 损坏的旧快照不会阻止 Core 发布新的完整快照。
            }
        }

        var published = new PlannerSnapshot
        {
            SchemaVersion = 1,
            Revision = Math.Max(previousRevision, snapshot.Revision) + 1,
            UpdatedAtUtc = DateTimeOffset.UtcNow,
            Boxes = snapshot.Boxes,
            Plan = snapshot.Plan,
            ReusablePlacements = snapshot.ReusablePlacements,
        };
        string temporaryPath = $"{fullPath}.{Environment.ProcessId}.{Guid.NewGuid():N}.tmp";
        try
        {
            using (var stream = new FileStream(
                       temporaryPath, FileMode.CreateNew, FileAccess.Write, FileShare.None,
                       bufferSize: 4096, FileOptions.WriteThrough))
            {
                JsonSerializer.Serialize(stream, published, JsonOptions);
                stream.Flush(flushToDisk: true);
            }
            File.Move(temporaryPath, fullPath, overwrite: true);
            return published;
        }
        finally
        {
            if (File.Exists(temporaryPath))
                File.Delete(temporaryPath);
        }
    }
}