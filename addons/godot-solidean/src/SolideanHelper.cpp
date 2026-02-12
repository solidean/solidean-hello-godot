#include "SolideanHelper.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cmath>

using namespace godot;

// C++17 fallback for std::numbers::pi (C++20)
namespace
{
constexpr double PI = 3.14159265358979323846;
}

// === Lifecycle ===

SolideanHelper::SolideanHelper()
{
    ensure_context_initialized();
}

SolideanHelper::~SolideanHelper() = default;

void SolideanHelper::clear_cache_entry(MeshInstance3D* mesh)
{
    if (mesh)
    {
        mesh_cache_.erase(mesh);
        if (mesh_cache_.empty())
            next_surface_id_ = 0;
    }
}

void SolideanHelper::ensure_context_initialized()
{
    if (context_ != nullptr)
        return;

    context_ = solidean::Context::create();
    if (!context_)
    {
        UtilityFunctions::push_error("SolideanHelper: Failed to create Solidean context");
        return;
    }

    exact_arithmetic_ = context_->createExactArithmetic(arithmetic_range_);
    if (!exact_arithmetic_)
    {
        UtilityFunctions::push_error("SolideanHelper: Failed to create exact arithmetic");
        return;
    }
}

// === Configuration ===

void SolideanHelper::set_arithmetic_range(float range)
{
    if (range <= 0.0f)
    {
        UtilityFunctions::push_warning("SolideanHelper: Invalid arithmetic range, must be > 0");
        return;
    }

    arithmetic_range_ = range;

    // Recreate exact arithmetic with new range
    if (context_)
        exact_arithmetic_ = context_->createExactArithmetic(arithmetic_range_);
}

float SolideanHelper::get_arithmetic_range() const
{
    return arithmetic_range_;
}

void SolideanHelper::set_normal_split_angle(double angle_degrees)
{
    normal_split_angle_ = std::max(0.0, std::min(angle_degrees, 180.0));
}

double SolideanHelper::get_normal_split_angle() const
{
    return normal_split_angle_;
}

// === Triangle Extraction ===

bool SolideanHelper::extract_triangles(MeshInstance3D* mesh_instance,
                                       std::vector<float>& out_vertices,
                                       std::vector<float>& out_normals,
                                       std::vector<int32_t>& out_indices)
{
    if (!mesh_instance)
    {
        UtilityFunctions::push_error("SolideanHelper: MeshInstance3D is null");
        return false;
    }

    static_assert(sizeof(real_t) == 4, "real_t must be 4 bytes (float)");

    Ref<Mesh> mesh = mesh_instance->get_mesh();
    if (!mesh.is_valid() || mesh->get_surface_count() == 0)
    {
        UtilityFunctions::push_warning("SolideanHelper: Mesh is invalid or has no surfaces");
        return false;
    }

    // Read vertex/normal/index data from the first surface
    Array const surface_arrays = mesh->surface_get_arrays(0);
    PackedVector3Array const vertices = surface_arrays[Mesh::ARRAY_VERTEX];
    PackedVector3Array const normals = surface_arrays[Mesh::ARRAY_NORMAL];

    if (vertices.size() != normals.size())
    {
        UtilityFunctions::push_error("SolideanHelper: Vertex and normal count mismatch");
        return false;
    }

    PackedInt32Array const indices = surface_arrays[Mesh::ARRAY_INDEX];
    Transform3D const transform = mesh_instance->get_global_transform();

    // Transform vertices to world space
    PackedVector3Array transformed_vertices;
    transformed_vertices.resize(vertices.size());
    for (int i = 0; i < vertices.size(); ++i)
        transformed_vertices[i] = transform.xform(vertices[i]);

    // Copy to output buffers
    out_vertices.resize(transformed_vertices.size() * 3);
    std::memcpy(out_vertices.data(), transformed_vertices.ptr(), transformed_vertices.size() * sizeof(Vector3));

    out_normals.resize(normals.size() * 3);
    std::memcpy(out_normals.data(), normals.ptr(), normals.size() * sizeof(Vector3));

    // Handle indexed vs non-indexed meshes
    if (indices.is_empty())
    {
        // Triangle soup, generate indices
        out_indices.reserve(vertices.size());
        for (int32_t i = 0; i < vertices.size(); i += 3)
        {
            out_indices.push_back(i);
            out_indices.push_back(i + 1);
            out_indices.push_back(i + 2);
        }
    }
    else
    {
        out_indices.resize(indices.size());
        std::memcpy(out_indices.data(), indices.ptr(), indices.size() * sizeof(int32_t));
    }

    return true;
}

// === Import/Export ===

bool SolideanHelper::import_from_godot(MeshInstance3D* mesh, int32_t surface_id, solidean::MeshType mesh_type)
{
    // Already cached, nothing to do
    if (mesh_cache_.count(mesh) > 0 && mesh_cache_[mesh].solidean_mesh != nullptr)
    {
        return true;
    }

    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<int32_t> indices;

    // Check for cached triangle data (avoids re-extraction from Godot arrays)
    bool has_cached_triangles = mesh_cache_.count(mesh) > 0 && mesh_cache_[mesh].cache_triangles
                             && !mesh_cache_[mesh].cached_vertices.empty();

    if (has_cached_triangles)
    {
        // Reuse cached triangles with transform delta
        auto const& cache = mesh_cache_[mesh];
        Transform3D current_transform = mesh->get_global_transform();
        Transform3D transform_delta = current_transform * cache.cached_transform.affine_inverse();

        // Copy cached triangle data
        vertices = cache.cached_vertices;
        normals = cache.cached_normals;
        indices = cache.cached_indices;

        // Apply transform delta to bring cached world-space data up to date
        for (size_t i = 0; i < vertices.size(); i += 3)
        {
            auto v = Vector3(vertices[i], vertices[i + 1], vertices[i + 2]);
            v = transform_delta.xform(v);
            vertices[i] = v.x;
            vertices[i + 1] = v.y;
            vertices[i + 2] = v.z;
        }

        for (size_t i = 0; i < normals.size(); i += 3)
        {
            auto n = Vector3(normals[i], normals[i + 1], normals[i + 2]);
            n = transform_delta.basis.xform(n);
            n = n.normalized();
            normals[i] = n.x;
            normals[i + 1] = n.y;
            normals[i + 2] = n.z;
        }
    }
    else
    {
        // Extract triangles from mesh
        if (!extract_triangles(mesh, vertices, normals, indices))
        {
            return false;
        }

        // Cache triangle data if requested
        if (mesh_cache_.count(mesh) > 0 && mesh_cache_[mesh].cache_triangles)
        {
            mesh_cache_[mesh].cached_vertices = vertices;
            mesh_cache_[mesh].cached_normals = normals;
            mesh_cache_[mesh].cached_indices = indices;
            mesh_cache_[mesh].cached_transform = mesh->get_global_transform();
        }
    }

    if (vertices.empty() || indices.empty())
    {
        UtilityFunctions::push_warning("SolideanHelper: Empty mesh, skipping import");
        return false;
    }

    // Reinterpret flat float/int arrays as typed solidean spans
    auto vertices_span = make_span<float, solidean::pos3>(vertices);
    auto triangles_span = make_span<int32_t, solidean::idxtri>(indices);

    // Import into solidean with mesh-type-dependent preprocessing
    try
    {
        auto solidean_mesh = context_->execute( //
            *exact_arithmetic_,
            [&](solidean::Operation& op)
            {
                auto mesh = op.importFromIndexedTrianglesF32WithID(vertices_span, triangles_span, surface_id, mesh_type);

                if (mesh_type == solidean::MeshType::Supersolid)
                {
                    mesh = op.selfUnion(mesh);
                }
                else if (mesh_type == solidean::MeshType::NonSupersolid)
                {
                    mesh = op.heal(mesh);
                    mesh = op.selfUnion(mesh);
                }

                return op.output(mesh);
            });

        // Store imported mesh in cache
        mesh_cache_[mesh].solidean_mesh = std::move(solidean_mesh);
        mesh_cache_[mesh].surface_id = surface_id;

        return true;
    }
    catch (std::exception const& e)
    {
        UtilityFunctions::push_error("SolideanHelper: Mesh import failed - ", e.what());
        return false;
    }
}

void SolideanHelper::set_empty_mesh(MeshInstance3D* mesh_ptr)
{
    // Replace current mesh with a blank ArrayMesh
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh_ptr->set_mesh(mesh);
}

void SolideanHelper::export_to_godot(MeshInstance3D* mesh)
{
    if (mesh_cache_.count(mesh) == 0 || !mesh_cache_[mesh].solidean_mesh)
    {
        UtilityFunctions::push_error("SolideanHelper: No cached mesh to export");
        return;
    }

    // Export solidean mesh to indexed triangles with per-triangle surface IDs
    auto const result_blob = context_->execute(
        *exact_arithmetic_, [&](solidean::Operation& op)
        { return op.exportToIndexedTrianglesF32WithID(op.input(*mesh_cache_[mesh].solidean_mesh)); });

    // Extract result geometry
    auto triangles = result_blob->getTrianglesIndexed();
    auto positions = result_blob->getPositionsF32();
    auto surface_ids = result_blob->getPrimitiveIDs();

    // Handle empty result
    if (positions.size() == 0)
    {
        set_empty_mesh(mesh);
        last_operation_stats_ = {0, 0};
        UtilityFunctions::print("SolideanHelper: Exported empty mesh");
        return;
    }

    // Copy positions
    std::vector<Vector3> vertices(positions.size());
    std::memcpy(vertices.data(), positions.data(), positions.size() * sizeof(Vector3));

    // Convert indices
    std::vector<Vector3i> indices;
    indices.reserve(triangles.size());
    for (auto const& tri : triangles)
    {
        // Swap winding order for Godot
        indices.push_back(Vector3i(tri.i1, tri.i0, tri.i2));
    }

    if (indices.empty())
    {
        set_empty_mesh(mesh);
        last_operation_stats_ = {0, 0};
        return;
    }

    // Split vertices at surface boundaries
    split_vertices_at_boundaries(vertices, indices, triangles, surface_ids);

    // Compute smooth normals
    auto normals = compute_normals(vertices, indices, normal_split_angle_);

    // Populate Godot arrays
    PackedVector3Array vertex_array;
    vertex_array.resize(vertices.size());
    std::memcpy(vertex_array.ptrw(), vertices.data(), vertices.size() * sizeof(Vector3));

    PackedInt32Array index_array;
    index_array.resize(indices.size() * 3);
    std::memcpy(index_array.ptrw(), indices.data(), indices.size() * sizeof(Vector3i));

    PackedVector3Array normal_array;
    normal_array.resize(normals.size());
    std::memcpy(normal_array.ptrw(), normals.data(), normals.size() * sizeof(Vector3));

    // Create mesh
    Array surfaces;
    surfaces.resize(Mesh::ARRAY_MAX);
    surfaces[Mesh::ARRAY_VERTEX] = vertex_array;
    surfaces[Mesh::ARRAY_NORMAL] = normal_array;
    surfaces[Mesh::ARRAY_INDEX] = index_array;

    Ref<ArrayMesh> array_mesh;
    array_mesh.instantiate();
    array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surfaces);

    // Assign result mesh and reset transform to identity (vertices are world-space)
    mesh->set_mesh(array_mesh);
    mesh->set_global_transform(Transform3D());

    // Store stats
    last_operation_stats_.num_vertices = static_cast<uint32_t>(vertices.size());
    last_operation_stats_.num_triangles = static_cast<uint32_t>(indices.size());
}

// === Boolean Operations ===

std::unique_ptr<solidean::Mesh> SolideanHelper::execute_boolean_operation(solidean::Mesh const& mesh_a,
                                                                          solidean::Mesh const& mesh_b,
                                                                          BooleanOp operation)
{
    return context_->execute( //
        *exact_arithmetic_,
        [&](solidean::Operation& op)
        {
            solidean::MeshOperand result;

            switch (operation)
            {
            case UNION: result = op.union_(op.input(mesh_a), op.input(mesh_b)); break;
            case INTERSECTION: result = op.intersection(op.input(mesh_a), op.input(mesh_b)); break;
            case DIFFERENCE: result = op.difference(op.input(mesh_a), op.input(mesh_b)); break;
            case DIFFERENCE_SYMMETRIC: result = op.differenceSymmetric(op.input(mesh_a), op.input(mesh_b)); break;
            default:
                UtilityFunctions::push_error("SolideanHelper: Invalid operation type");
                return op.output(op.input(mesh_a));
            }

            return op.output(result);
        });
}

void SolideanHelper::operation(MeshInstance3D* mesh_a,
                               MeshInstance3D* mesh_b,
                               int op_mode,
                               bool clear_a,
                               bool clear_b,
                               MeshInstance3D* result_mesh,
                               int mesh_type,
                               bool cache_triangles_b)
{
    if (!mesh_a)
    {
        UtilityFunctions::push_error("SolideanHelper: Null mesh A pointer in operation");
        return;
    }

    if (!context_ || !exact_arithmetic_)
    {
        UtilityFunctions::push_error("SolideanHelper: Context not initialized");
        return;
    }

    // Clear mesh A cache if requested (when mesh geometry changes)
    if (clear_a)
        mesh_cache_.erase(mesh_a);

    // Reset stats
    last_operation_stats_ = {};

    // Convert Godot enum to solidean enum
    auto solidean_mesh_type = solidean::MeshType(mesh_type);

    // Enable triangle caching for mesh_b if requested
    if (mesh_b && cache_triangles_b)
        mesh_cache_[mesh_b].cache_triangles = true;

    // Import meshes if not already cached
    uint64_t t_start_imports = Time::get_singleton()->get_ticks_usec();

    for (auto* mesh : {mesh_a, mesh_b})
    {
        if (!mesh)
            continue;

        if (mesh_cache_.count(mesh) == 0 || !mesh_cache_[mesh].solidean_mesh)
        {
            int32_t const surface_id = next_surface_id_++;

            if (!import_from_godot(mesh, surface_id, solidean_mesh_type))
            {
                UtilityFunctions::push_error("SolideanHelper: Failed to import mesh");
                return;
            }
        }
    }

    last_operation_stats_.import_time_ms = (Time::get_singleton()->get_ticks_usec() - t_start_imports) / 1000.0;

    // Determine output mesh
    MeshInstance3D* output_mesh = result_mesh ? result_mesh : mesh_a;

    // May be nullptr if only performing import (+ self-union)
    if (mesh_b)
    {
        // Perform operation
        try
        {
            uint64_t t_start_execute = Time::get_singleton()->get_ticks_usec();
            auto result = execute_boolean_operation(*mesh_cache_[mesh_a].solidean_mesh,
                                                    *mesh_cache_[mesh_b].solidean_mesh, static_cast<BooleanOp>(op_mode));
            last_operation_stats_.execute_time_ms = (Time::get_singleton()->get_ticks_usec() - t_start_execute) / 1000.0;
            mesh_cache_[output_mesh].solidean_mesh = std::move(result);
        }
        catch (std::exception const& e)
        {
            UtilityFunctions::push_error("SolideanHelper: Boolean operation failed - ", e.what());
            return;
        }
    }

    // Export
    try
    {
        uint64_t t_start_export = Time::get_singleton()->get_ticks_usec();
        export_to_godot(output_mesh);
        last_operation_stats_.export_time_ms = (Time::get_singleton()->get_ticks_usec() - t_start_export) / 1000.0;
    }
    catch (std::exception const& e)
    {
        UtilityFunctions::push_error("SolideanHelper: Export failed - ", e.what());
        return;
    }

    // Clear solidean mesh for B if requested (for dynamic transforms)
    if (clear_b && mesh_b)
    {
        if (cache_triangles_b)
        {
            // Keep triangle cache, only clear the solidean mesh
            mesh_cache_[mesh_b].solidean_mesh.reset();
            mesh_cache_[mesh_b].surface_id = -1;
        }
        else
        {
            // Clear entire cache entry
            mesh_cache_.erase(mesh_b);
        }
    }
}

// === Normal Computation ===
// NOTE: the normal computation below is not optimized yet, but it is still much faster than
//       Godot's physics mesh import, which currently dominates the overall pipeline cost.

void SolideanHelper::split_vertices_at_boundaries(std::vector<Vector3>& vertices,
                                                  std::vector<Vector3i>& indices,
                                                  solidean::span<solidean::idxtri const> triangles,
                                                  solidean::span<solidean::tracking_id const> surface_ids)
{
    size_t const num_vertices = vertices.size();
    size_t const num_triangles = indices.size();

    // Track which surface IDs reference each vertex
    std::vector<std::set<int32_t>> vertex_surfaces(num_vertices);

    for (size_t i = 0; i < num_triangles; ++i)
    {
        auto const& tri = indices[i];
        int32_t const sid = surface_ids[i].surface_id();
        vertex_surfaces[tri.x].insert(sid);
        vertex_surfaces[tri.y].insert(sid);
        vertex_surfaces[tri.z].insert(sid);
    }

    // Create one duplicate per surface (beyond the first) for each multi-surface
    // vertex remap[v][sid] = new vertex index for vertex v when used by surface
    // sid
    std::vector<std::map<int32_t, int>> vertex_remap(num_vertices);

    for (size_t v = 0; v < num_vertices; ++v)
    {
        if (vertex_surfaces[v].size() <= 1)
            continue;

        auto it = vertex_surfaces[v].begin();
        int32_t const first_sid = *it;
        ++it;

        // First surface keeps the original vertex
        vertex_remap[v][first_sid] = static_cast<int>(v);

        // Remaining surfaces get duplicates
        for (; it != vertex_surfaces[v].end(); ++it)
        {
            int32_t const sid = *it;
            int const new_idx = static_cast<int>(vertices.size());
            vertices.push_back(vertices[v]);
            vertex_remap[v][sid] = new_idx;
        }
    }

    // Remap triangle indices to use the correct vertex copy per surface
    for (size_t i = 0; i < num_triangles; ++i)
    {
        int32_t const sid = surface_ids[i].surface_id();
        auto& tri = indices[i];

        if (!vertex_remap[tri.x].empty())
            tri.x = vertex_remap[tri.x][sid];
        if (!vertex_remap[tri.y].empty())
            tri.y = vertex_remap[tri.y][sid];
        if (!vertex_remap[tri.z].empty())
            tri.z = vertex_remap[tri.z][sid];
    }
}

std::vector<Vector3> SolideanHelper::compute_normals(std::vector<Vector3>& vertices,
                                                     std::vector<Vector3i>& indices,
                                                     double max_angle_deviation_degrees)
{
    size_t const num_verts = vertices.size();
    size_t const num_faces = indices.size();

    // Compute face normals (unnormalized = area-weighted, and unit)
    std::vector<Vector3> face_normals(num_faces);
    std::vector<Vector3> face_normals_unit(num_faces);

    for (size_t i = 0; i < num_faces; ++i)
    {
        auto const& v0 = vertices[indices[i].x];
        auto const& v1 = vertices[indices[i].y];
        auto const& v2 = vertices[indices[i].z];
        auto normal = (v1 - v2).cross(v0 - v1);
        face_normals[i] = normal;
        auto len = normal.length();
        face_normals_unit[i] = len > 0.0f ? normal / len : Vector3();
    }

    // Fast path: no angle limit, pure area-weighted smooth normals
    if (max_angle_deviation_degrees >= 180.0)
    {
        std::vector<Vector3> normals(num_verts, Vector3());
        for (size_t i = 0; i < num_faces; ++i)
        {
            normals[indices[i].x] += face_normals[i];
            normals[indices[i].y] += face_normals[i];
            normals[indices[i].z] += face_normals[i];
        }
        for (auto& n : normals)
            n = n.normalized();
        return normals;
    }

    // Build vertex-to-face adjacency (CSR format)
    std::vector<int> offsets(num_verts + 1, 0);
    for (size_t i = 0; i < num_faces; ++i)
    {
        ++offsets[indices[i].x];
        ++offsets[indices[i].y];
        ++offsets[indices[i].z];
    }
    for (size_t i = 1; i <= num_verts; ++i)
        offsets[i] += offsets[i - 1];

    std::vector<int> adj(offsets.back());
    for (size_t i = 0; i < num_faces; ++i)
    {
        adj[--offsets[indices[i].x]] = static_cast<int>(i);
        adj[--offsets[indices[i].y]] = static_cast<int>(i);
        adj[--offsets[indices[i].z]] = static_cast<int>(i);
    }

    double const cos_limit = std::cos(max_angle_deviation_degrees * PI / 180.0);

    // Per-vertex smooth normals with angle-based splitting
    std::vector<Vector3> normals(num_verts, Vector3());

    for (size_t v = 0; v < num_verts; ++v)
    {
        int const adj_start = offsets[v];
        int const adj_end = offsets[v + 1];
        int const adj_count = adj_end - adj_start;

        if (adj_count == 0)
        {
            normals[v] = Vector3(0, 1, 0);
            continue;
        }

        if (adj_count == 1)
        {
            normals[v] = face_normals_unit[adj[adj_start]];
            continue;
        }

        // Group adjacent faces by normal compatibility (transitive flood-fill)
        std::vector<int> group(adj_count, -1);
        int num_groups = 0;

        for (int k = 0; k < adj_count; ++k)
        {
            if (group[k] >= 0)
                continue;
            int const g = num_groups++;
            group[k] = g;

            bool changed = true;
            while (changed)
            {
                changed = false;
                for (int m = 0; m < adj_count; ++m)
                {
                    if (group[m] >= 0)
                        continue;
                    int const face_m = adj[adj_start + m];
                    for (int n = 0; n < adj_count; ++n)
                    {
                        if (group[n] != g)
                            continue;
                        if (face_normals_unit[face_m].dot(face_normals_unit[adj[adj_start + n]]) >= cos_limit)
                        {
                            group[m] = g;
                            changed = true;
                            break;
                        }
                    }
                }
            }
        }

        // All faces compatible -> simple average, no splitting needed
        if (num_groups == 1)
        {
            Vector3 n;
            for (int k = 0; k < adj_count; ++k)
                n += face_normals[adj[adj_start + k]];
            normals[v] = n.normalized();
            continue;
        }

        // Multiple groups: first group keeps original vertex, rest get duplicates
        for (int g = 0; g < num_groups; ++g)
        {
            Vector3 smooth_normal;
            for (int k = 0; k < adj_count; ++k)
            {
                if (group[k] == g)
                    smooth_normal += face_normals[adj[adj_start + k]];
            }
            smooth_normal = smooth_normal.normalized();

            if (g == 0)
            {
                normals[v] = smooth_normal;
            }
            else
            {
                int const new_idx = static_cast<int>(vertices.size());
                vertices.push_back(vertices[v]);
                normals.push_back(smooth_normal);

                // Update triangle indices for faces in this group
                for (int k = 0; k < adj_count; ++k)
                {
                    if (group[k] != g)
                        continue;
                    int const face = adj[adj_start + k];
                    auto& tri = indices[face];
                    if (tri.x == static_cast<int>(v))
                        tri.x = new_idx;
                    else if (tri.y == static_cast<int>(v))
                        tri.y = new_idx;
                    else
                        tri.z = new_idx;
                }
            }
        }
    }

    return normals;
}

// === Stats ===

uint32_t SolideanHelper::getNumVerticesLastOperation() const
{
    return last_operation_stats_.num_vertices;
}

uint32_t SolideanHelper::getNumTrianglesLastOperation() const
{
    return last_operation_stats_.num_triangles;
}

double SolideanHelper::getImportTimeMs() const
{
    return last_operation_stats_.import_time_ms;
}

double SolideanHelper::getExecuteTimeMs() const
{
    return last_operation_stats_.execute_time_ms;
}

double SolideanHelper::getExportTimeMs() const
{
    return last_operation_stats_.export_time_ms;
}

// === OBJ Export ===

bool SolideanHelper::export_to_obj(MeshInstance3D* mesh, String const& path)
{
    if (!mesh)
    {
        UtilityFunctions::push_error("SolideanHelper: Null mesh in export_to_obj");
        return false;
    }

    Ref<Mesh> godot_mesh = mesh->get_mesh();
    if (!godot_mesh.is_valid() || godot_mesh->get_surface_count() == 0)
    {
        UtilityFunctions::push_error("SolideanHelper: Invalid or empty mesh in export_to_obj");
        return false;
    }

    Ref<FileAccess> file = FileAccess::open(path, FileAccess::WRITE);
    if (!file.is_valid())
    {
        UtilityFunctions::push_error("SolideanHelper: Failed to open file: ", path);
        return false;
    }

    file->store_line("# Exported by SolideanHelper");

    // Read first surface data
    Array const surface_arrays = godot_mesh->surface_get_arrays(0);
    PackedVector3Array const vertices = surface_arrays[Mesh::ARRAY_VERTEX];
    PackedVector3Array const normals = surface_arrays[Mesh::ARRAY_NORMAL];
    PackedInt32Array const indices = surface_arrays[Mesh::ARRAY_INDEX];

    // Write vertices
    for (int i = 0; i < vertices.size(); ++i)
    {
        Vector3 const& v = vertices[i];
        file->store_line(vformat("v %f %f %f", v.x, v.y, v.z));
    }

    // Write normals
    for (int i = 0; i < normals.size(); ++i)
    {
        Vector3 const& n = normals[i];
        file->store_line(vformat("vn %f %f %f", n.x, n.y, n.z));
    }

    // Write faces (OBJ indices are 1-based)
    if (indices.is_empty())
    {
        // Triangle soup -> every 3 vertices form a face
        for (int i = 0; i < vertices.size(); i += 3)
        {
            file->store_line(vformat("f %d//%d %d//%d %d//%d", i + 1, i + 1, i + 2, i + 2, i + 3, i + 3));
        }
    }
    else
    {
        // Indexed mesh
        for (int i = 0; i < indices.size(); i += 3)
        {
            int i0 = indices[i] + 1;
            int i1 = indices[i + 1] + 1;
            int i2 = indices[i + 2] + 1;
            file->store_line(vformat("f %d//%d %d//%d %d//%d", i0, i0, i1, i1, i2, i2));
        }
    }

    UtilityFunctions::print("SolideanHelper: Exported mesh to ", path, " (", vertices.size(), " vertices, ",
                            indices.size() / 3, " triangles)");
    return true;
}

// === Binding ===

void SolideanHelper::_bind_methods()
{
    // BooleanOp enum constants
    BIND_ENUM_CONSTANT(UNION);
    BIND_ENUM_CONSTANT(INTERSECTION);
    BIND_ENUM_CONSTANT(DIFFERENCE);
    BIND_ENUM_CONSTANT(DIFFERENCE_SYMMETRIC);

    // MeshType enum constants
    BIND_ENUM_CONSTANT(SOLID);
    BIND_ENUM_CONSTANT(SUPERSOLID);
    BIND_ENUM_CONSTANT(NON_SUPERSOLID);

    // Methods
    ClassDB::bind_method(D_METHOD("operation", "mesh_a", "mesh_b", "op_mode", "clear_a", "clear_b", "result_mesh",
                                  "mesh_type", "cache_triangles_b"),
                         &SolideanHelper::operation, DEFVAL(UNION), DEFVAL(false), DEFVAL(true), DEFVAL(nullptr),
                         DEFVAL(SOLID), DEFVAL(false));

    ClassDB::bind_method(D_METHOD("getNumVerticesLastOperation"), &SolideanHelper::getNumVerticesLastOperation);

    ClassDB::bind_method(D_METHOD("getNumTrianglesLastOperation"), &SolideanHelper::getNumTrianglesLastOperation);

    ClassDB::bind_method(D_METHOD("getImportTimeMs"), &SolideanHelper::getImportTimeMs);
    ClassDB::bind_method(D_METHOD("getExecuteTimeMs"), &SolideanHelper::getExecuteTimeMs);
    ClassDB::bind_method(D_METHOD("getExportTimeMs"), &SolideanHelper::getExportTimeMs);

    ClassDB::bind_method(D_METHOD("clear_cache_entry", "mesh"), &SolideanHelper::clear_cache_entry);

    ClassDB::bind_method(D_METHOD("export_to_obj", "mesh", "path"), &SolideanHelper::export_to_obj);

    ClassDB::bind_method(D_METHOD("set_arithmetic_range", "range"), &SolideanHelper::set_arithmetic_range);

    ClassDB::bind_method(D_METHOD("get_arithmetic_range"), &SolideanHelper::get_arithmetic_range);

    ClassDB::bind_method(D_METHOD("set_normal_split_angle", "angle_degrees"), &SolideanHelper::set_normal_split_angle);

    ClassDB::bind_method(D_METHOD("get_normal_split_angle"), &SolideanHelper::get_normal_split_angle);

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "arithmetic_range"), "set_arithmetic_range", "get_arithmetic_range");

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "normal_split_angle"), "set_normal_split_angle", "get_normal_split_angle");
}

VARIANT_ENUM_CAST(SolideanHelper::BooleanOp);
VARIANT_ENUM_CAST(SolideanHelper::MeshType);
