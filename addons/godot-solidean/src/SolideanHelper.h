#pragma once

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/vector3i.hpp>

#include <solidean.hh>

#include <map>
#include <memory>
#include <optional>

namespace godot
{

class MeshInstance3D;

/// Godot wrapper around the solidean mesh-boolean library.
/// Manages import/export between Godot MeshInstance3D nodes and solidean's
/// internal representation, caching imported meshes to amortize repeated
/// operations. Designed as a showcase integration of solidean into Godot.
class SolideanHelper : public Object
{
    GDCLASS(SolideanHelper, Object)

public:
    static constexpr float DEFAULT_ARITHMETIC_RANGE = 10.0f;
    static constexpr double DEFAULT_NORMAL_SPLIT_ANGLE = 30.0;
    static constexpr solidean::MeshType DEFAULT_MESH_TYPE = solidean::MeshType::Solid;

    /// Maps to solidean's boolean operation types.
    enum BooleanOp
    {
        UNION,
        INTERSECTION,
        DIFFERENCE,
        DIFFERENCE_SYMMETRIC
    };

    /// Controls how solidean interprets imported geometry.
    /// SOLID: well-formed closed mesh (fastest).
    /// SUPERSOLID: may self-intersect; resolved via self-union.
    /// NON_SUPERSOLID: potentially degenerate; healed then self-unioned.
    enum MeshType
    {
        SOLID = 0,
        SUPERSOLID = 1,
        NON_SUPERSOLID = 2
    };

    /// Timing and mesh statistics from the most recent operation() call.
    struct OperationStats
    {
        uint32_t num_vertices = 0;
        uint32_t num_triangles = 0;
        double import_time_ms = 0.0;
        double execute_time_ms = 0.0;
        double export_time_ms = 0.0;
    };

    SolideanHelper();
    ~SolideanHelper();

    /// Performs a boolean operation between two meshes.
    /// Both meshes are imported into solidean on first use and cached by pointer.
    /// Pass clear_a/clear_b to invalidate cached geometry when the mesh changes.
    /// If result_mesh is null, the result replaces mesh_a's geometry.
    /// When cache_triangles_b is true, raw triangle data for mesh_b is kept so
    /// that only the transform delta needs recomputing on subsequent calls —
    /// useful when mesh_b moves each frame but its geometry is static.
    void operation(MeshInstance3D* mesh_a,
                   MeshInstance3D* mesh_b,
                   int op_mode = UNION,
                   bool clear_a = false,
                   bool clear_b = true,
                   MeshInstance3D* result_mesh = nullptr,
                   int mesh_type = SOLID,
                   bool cache_triangles_b = false);

    uint32_t getNumVerticesLastOperation() const;
    uint32_t getNumTrianglesLastOperation() const;
    double getImportTimeMs() const;
    double getExecuteTimeMs() const;
    double getExportTimeMs() const;

    /// Writes the first surface of a MeshInstance3D to an OBJ file.
    /// Returns false on null mesh, missing geometry, or file I/O failure.
    bool export_to_obj(MeshInstance3D* mesh, String const& path);

    /// Removes a mesh's cache entry.
    /// Must be called before freeing a MeshInstance3D that was used in an
    /// operation, to avoid dangling pointers and leaked memory in the cache.
    void clear_cache_entry(MeshInstance3D* mesh);

    /// Sets the coordinate range for solidean's exact arithmetic kernel.
    /// All vertex coordinates must fit within [-range, range] on every axis.
    /// Triggers re-creation of the exact arithmetic instance.
    void set_arithmetic_range(float range);
    float get_arithmetic_range() const;

    /// Angle threshold (degrees) for smooth-normal splitting during export.
    /// Adjacent faces whose normals deviate by more than this get independent
    /// (hard-edge) normals. Clamped to [0, 180].
    void set_normal_split_angle(double angle_degrees);
    double get_normal_split_angle() const;

private:
    /// Per-mesh cache entry. Stores the solidean mesh and, optionally, the raw
    /// triangle data extracted from Godot so it can be re-transformed cheaply
    /// without re-extracting from the Godot mesh arrays.
    struct MeshData
    {
        std::unique_ptr<solidean::Mesh> solidean_mesh;
        int32_t surface_id = -1;

        bool cache_triangles = false;
        std::vector<float> cached_vertices;
        std::vector<float> cached_normals;
        std::vector<int32_t> cached_indices;
        Transform3D cached_transform;
    };

    std::unique_ptr<solidean::Context> context_;
    std::unique_ptr<solidean::ExactArithmetic> exact_arithmetic_;

    /// Keyed by raw MeshInstance3D pointer. Entries must be removed manually
    /// via clear_cache_entry() before the node is freed.
    std::map<MeshInstance3D*, MeshData> mesh_cache_;

    OperationStats last_operation_stats_;
    float arithmetic_range_ = DEFAULT_ARITHMETIC_RANGE;
    double normal_split_angle_ = DEFAULT_NORMAL_SPLIT_ANGLE;

    /// Monotonically increasing ID assigned to each newly imported mesh.
    /// Solidean uses these to track which original surface produced each output
    /// triangle, enabling per-surface normal splitting at boundaries.
    int32_t next_surface_id_ = 0;

    /// Reinterprets a flat std::vector<FromT> as a solidean::span<ToT const>.
    template <typename FromT, typename ToT>
    static solidean::span<ToT const> make_span(std::vector<FromT> const& data)
    {
        static_assert(sizeof(ToT) % sizeof(FromT) == 0, "ToT size must be multiple of FromT size");
        size_t const total_size = data.size() * sizeof(FromT);
        return solidean::span<ToT const>(reinterpret_cast<ToT const*>(data.data()), total_size / sizeof(ToT));
    }

    /// Extracts world-space vertex positions, normals, and triangle indices
    /// from the first surface of a MeshInstance3D.
    bool extract_triangles(MeshInstance3D* mesh_instance,
                           std::vector<float>& out_vertices,
                           std::vector<float>& out_normals,
                           std::vector<int32_t>& out_indices);

    /// Imports a Godot mesh into solidean, applying mesh_type-dependent
    /// preprocessing (self-union for Supersolid, heal + self-union for
    /// NonSupersolid). No-ops if the mesh is already cached.
    bool import_from_godot(MeshInstance3D* mesh, int32_t surface_id, solidean::MeshType mesh_type = DEFAULT_MESH_TYPE);

    /// Exports the cached solidean mesh back into a Godot ArrayMesh, including
    /// surface-boundary vertex splitting and smooth normal computation.
    /// Resets the mesh's global transform to identity (vertices are world-space).
    void export_to_godot(MeshInstance3D* mesh);

    /// Duplicates vertices shared across different surface IDs so each surface
    /// can have independent normals at the boundary.
    void split_vertices_at_boundaries(std::vector<Vector3>& vertices,
                                      std::vector<Vector3i>& indices,
                                      solidean::span<solidean::idxtri const> triangles,
                                      solidean::span<solidean::tracking_id const> surface_ids);

    /// Computes area-weighted smooth vertex normals with angle-based splitting.
    /// Faces deviating by more than max_angle_deviation_degrees are placed in
    /// separate smoothing groups; vertices at group boundaries are duplicated
    /// (appended to vertices/indices) so each group gets its own normal.
    std::vector<Vector3> compute_normals(std::vector<Vector3>& vertices,
                                         std::vector<Vector3i>& indices,
                                         double max_angle_deviation_degrees);

    /// Assigns an empty ArrayMesh — used when a boolean op yields no geometry.
    void set_empty_mesh(MeshInstance3D* mesh);

    /// Lazily initializes the solidean Context and ExactArithmetic instances.
    void ensure_context_initialized();

    /// Dispatches to the appropriate solidean boolean operation.
    std::unique_ptr<solidean::Mesh> execute_boolean_operation(solidean::Mesh const& mesh_a,
                                                              solidean::Mesh const& mesh_b,
                                                              BooleanOp operation);

protected:
    static void _bind_methods();
};

} // namespace godot
