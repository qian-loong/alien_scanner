#include "perception_map_update/MerklePatricia.hpp"

#include "CanonicalEncoding.hpp"
#include "Sha256DigestSink.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        using Clock = std::chrono::steady_clock;
        constexpr std::size_t kCoordinateBits = kMerkleCoordinateKeyBytes * 8U;

        std::uint64_t elapsed_ns(Clock::time_point begin, Clock::time_point end) noexcept
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count());
        }

        using DigestSink = Internal::Sha256DigestSink;

        void write_domain(
                DigestSink & sink,
                const char * domain,
                std::uint16_t node_encoding_version)
        {
            Encoding::write_string(sink, domain);
            Encoding::write_u16(sink, kMerkleContentIdentityVersion);
            Encoding::write_u16(sink, node_encoding_version);
            Encoding::write_u8(sink, static_cast<std::uint8_t>(HashAlgorithm::Sha256));
        }

        void write_descriptor(DigestSink & sink, const ContentIdentityDescriptor & descriptor)
        {
            Encoding::write_u16(sink, static_cast<std::uint16_t>(descriptor.scheme));
            Encoding::write_u32(sink, descriptor.chunk_edge);
            Encoding::write_u16(sink, descriptor.coordinate_key_version);
            Encoding::write_u16(sink, descriptor.node_encoding_version);
        }

        void write_key(DigestSink & sink, const MerkleCoordinateKey & key)
        {
            sink.append(key.bytes.data(), key.bytes.size());
        }

        std::uint64_t sign_flip(std::int64_t value) noexcept
        {
            return static_cast<std::uint64_t>(value) ^ (std::uint64_t {1U} << 63U);
        }

        void write_key_axis(
                std::array<std::uint8_t, kMerkleCoordinateKeyBytes> & bytes,
                std::size_t offset,
                std::int64_t value) noexcept
        {
            const auto encoded = sign_flip(value);
            for(std::size_t index = 0U; index < sizeof(encoded); ++index) {
                bytes[offset + index] = static_cast<std::uint8_t>(
                        (encoded >> ((sizeof(encoded) - index - 1U) * 8U)) & 0xffU);
            }
        }

        bool bit_at(const MerkleCoordinateKey & key, std::size_t bit) noexcept
        {
            return (key.bytes[bit / 8U] & (std::uint8_t {0x80U} >> (bit % 8U))) != 0U;
        }

        std::size_t first_different_bit(
                const MerkleCoordinateKey & left,
                const MerkleCoordinateKey & right) noexcept
        {
            for(std::size_t byte = 0U; byte < left.bytes.size(); ++byte) {
                const auto different = static_cast<std::uint8_t>(
                        left.bytes[byte] ^ right.bytes[byte]);
                if(different == 0U) {
                    continue;
                }
                for(std::size_t bit = 0U; bit < 8U; ++bit) {
                    if((different & (std::uint8_t {0x80U} >> bit)) != 0U) {
                        return byte * 8U + bit;
                    }
                }
            }
            return kCoordinateBits;
        }

        struct Node;
        using NodePtr = std::shared_ptr<const Node>;

        struct Node {
            bool leaf = false;
            MerkleCoordinateKey key;
            Hash256 hash {};
            ChunkCoordinate coordinate;
            std::size_t cell_count = 0U;
            std::uint16_t branch_bit = 0U;
            NodePtr left;
            NodePtr right;
        };

        struct BuildContext {
            MerkleTreeMetrics metrics;
        };

        Hash256 hash_empty(const ContentIdentityDescriptor & descriptor)
        {
            DigestSink sink;
            write_domain(sink, "alien-scanner/map-merkle-empty/v2", descriptor.node_encoding_version);
            return sink.finish();
        }

        Hash256 hash_leaf(
                const ContentIdentityDescriptor & descriptor,
                const ChunkCoordinate & coordinate,
                const std::vector<CanonicalCell> & cells)
        {
            const auto key = merkle_coordinate_key(coordinate);
            DigestSink sink;
            write_domain(sink, "alien-scanner/map-merkle-leaf/v2", descriptor.node_encoding_version);
            write_key(sink, key);
            Encoding::write_i64(sink, coordinate.x);
            Encoding::write_i64(sink, coordinate.y);
            Encoding::write_i64(sink, coordinate.z);
            Encoding::write_u64(sink, static_cast<std::uint64_t>(cells.size()));
            for(const auto & cell : cells) {
                Encoding::write_cell(sink, cell);
            }
            return sink.finish();
        }

        Hash256 hash_branch(
                const ContentIdentityDescriptor & descriptor,
                std::uint16_t branch_bit,
                const NodePtr & left,
                const NodePtr & right)
        {
            DigestSink sink;
            write_domain(sink, "alien-scanner/map-merkle-branch/v2", descriptor.node_encoding_version);
            Encoding::write_u16(sink, branch_bit);
            Encoding::write_hash(sink, left->hash);
            Encoding::write_hash(sink, right->hash);
            return sink.finish();
        }

        Hash256 hash_content(
                const ContentIdentityDescriptor & descriptor,
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                std::size_t total_cell_count,
                const Hash256 & trie_root)
        {
            DigestSink sink;
            write_domain(sink, "alien-scanner/map-content/merkle-v2", descriptor.node_encoding_version);
            write_descriptor(sink, descriptor);
            Encoding::write_identity(sink, source);
            Encoding::write_hash(sink, geometry_fingerprint);
            Encoding::write_u64(sink, static_cast<std::uint64_t>(total_cell_count));
            Encoding::write_hash(sink, trie_root);
            return sink.finish();
        }

        NodePtr make_leaf(
                const ContentIdentityDescriptor & descriptor,
                const ChunkCoordinate & coordinate,
                const std::vector<CanonicalCell> & cells,
                BuildContext & context)
        {
            auto node = std::make_shared<Node>();
            node->leaf = true;
            node->coordinate = coordinate;
            node->key = merkle_coordinate_key(coordinate);
            node->cell_count = cells.size();
            const auto hash_begin = Clock::now();
            node->hash = hash_leaf(descriptor, coordinate, cells);
            const auto hash_end = Clock::now();
            ++context.metrics.allocated_nodes;
            context.metrics.candidate_owned_bytes += sizeof(Node);
            ++context.metrics.leaf_hashes;
            context.metrics.leaf_hash_ns += elapsed_ns(hash_begin, hash_end);
            return node;
        }

        NodePtr make_branch(
                const ContentIdentityDescriptor & descriptor,
                std::size_t branch_bit,
                NodePtr left,
                NodePtr right,
                BuildContext & context)
        {
            if(!left || !right || branch_bit >= kCoordinateBits) {
                throw std::logic_error("invalid Merkle Patricia branch");
            }
            auto node = std::make_shared<Node>();
            node->leaf = false;
            node->branch_bit = static_cast<std::uint16_t>(branch_bit);
            node->left = std::move(left);
            node->right = std::move(right);
            node->key = node->left->key;
            const auto hash_begin = Clock::now();
            node->hash = hash_branch(
                    descriptor,
                    node->branch_bit,
                    node->left,
                    node->right);
            const auto hash_end = Clock::now();
            ++context.metrics.allocated_nodes;
            context.metrics.candidate_owned_bytes += sizeof(Node);
            ++context.metrics.branch_hashes;
            context.metrics.branch_hash_ns += elapsed_ns(hash_begin, hash_end);
            return node;
        }

        struct ChunkLeaf {
            ChunkCoordinate coordinate;
            std::vector<CanonicalCell> cells;
        };

        bool validate_cells(
                const std::vector<CanonicalCell> & cells,
                std::string & diagnostic)
        {
            for(std::size_t index = 0U; index < cells.size(); ++index) {
                if(cells[index].state != CellState::Free
                   && cells[index].state != CellState::Occupied) {
                    diagnostic = "Merkle chunk has an invalid cell state";
                    return false;
                }
                if(index > 0U && !(cells[index - 1U].index < cells[index].index)) {
                    diagnostic = "Merkle cells are not strictly ordered";
                    return false;
                }
            }
            return true;
        }

        template<typename Visit>
        bool collect_chunks(
                const Visit & visit,
                std::uint32_t chunk_edge,
                std::vector<ChunkLeaf> & result,
                std::size_t & total_cell_count,
                std::string & diagnostic)
        {
            std::map<ChunkCoordinate, std::vector<CanonicalCell>> grouped;
            std::optional<VoxelIndex> previous;
            bool valid = true;
            total_cell_count = 0U;
            visit([&](const CanonicalCell & cell) {
                if(!valid) {
                    return;
                }
                if(cell.state != CellState::Free && cell.state != CellState::Occupied) {
                    diagnostic = "Merkle chunk has an invalid cell state";
                    valid = false;
                    return;
                }
                if(previous.has_value() && !(previous.value() < cell.index)) {
                    diagnostic = "Merkle cells are not strictly ordered";
                    valid = false;
                    return;
                }
                const auto address = locate_chunk(cell.index, chunk_edge);
                if(!address) {
                    diagnostic = address.diagnostic;
                    valid = false;
                    return;
                }
                grouped[address.address.chunk].push_back(cell);
                previous = cell.index;
                ++total_cell_count;
            });
            if(!valid) {
                return false;
            }
            result.reserve(grouped.size());
            for(auto & entry : grouped) {
                result.push_back({entry.first, std::move(entry.second)});
            }
            return true;
        }

        NodePtr build_range(
                const ContentIdentityDescriptor & descriptor,
                const std::vector<NodePtr> & leaves,
                std::size_t begin,
                std::size_t end,
                BuildContext & context)
        {
            if(end - begin == 1U) {
                return leaves[begin];
            }
            const auto bit = first_different_bit(
                    leaves[begin]->key,
                    leaves[end - 1U]->key);
            if(bit >= kCoordinateBits) {
                throw std::logic_error("duplicate Merkle Patricia key");
            }
            std::size_t split = begin;
            while(split < end && !bit_at(leaves[split]->key, bit)) {
                ++split;
            }
            if(split == begin || split == end) {
                throw std::logic_error("invalid Merkle Patricia split");
            }
            return make_branch(
                    descriptor,
                    bit,
                    build_range(descriptor, leaves, begin, split, context),
                    build_range(descriptor, leaves, split, end, context),
                    context);
        }

        const Node * find_leaf(const NodePtr & node, const MerkleCoordinateKey & key) noexcept
        {
            if(!node) {
                return nullptr;
            }
            if(node->leaf) {
                return node->key == key ? node.get() : nullptr;
            }
            return find_leaf(bit_at(key, node->branch_bit) ? node->right : node->left, key);
        }

        NodePtr insert_node(
                const ContentIdentityDescriptor & descriptor,
                const NodePtr & node,
                NodePtr leaf,
                BuildContext & context)
        {
            if(!node) {
                return leaf;
            }
            if(node->leaf) {
                if(node->key == leaf->key) {
                    return leaf;
                }
                const auto bit = first_different_bit(node->key, leaf->key);
                if(bit_at(leaf->key, bit)) {
                    return make_branch(descriptor, bit, node, std::move(leaf), context);
                }
                return make_branch(descriptor, bit, std::move(leaf), node, context);
            }

            const auto bit = first_different_bit(node->key, leaf->key);
            if(bit < node->branch_bit) {
                if(bit_at(leaf->key, bit)) {
                    return make_branch(descriptor, bit, node, std::move(leaf), context);
                }
                return make_branch(descriptor, bit, std::move(leaf), node, context);
            }

            if(bit_at(leaf->key, node->branch_bit)) {
                auto child = insert_node(descriptor, node->right, std::move(leaf), context);
                ++context.metrics.path_nodes_rebuilt;
                return make_branch(descriptor, node->branch_bit, node->left, std::move(child), context);
            }
            auto child = insert_node(descriptor, node->left, std::move(leaf), context);
            ++context.metrics.path_nodes_rebuilt;
            return make_branch(descriptor, node->branch_bit, std::move(child), node->right, context);
        }

        NodePtr remove_node(
                const ContentIdentityDescriptor & descriptor,
                const NodePtr & node,
                const MerkleCoordinateKey & key,
                bool & removed,
                BuildContext & context)
        {
            if(!node) {
                return node;
            }
            if(node->leaf) {
                if(node->key != key) {
                    return node;
                }
                removed = true;
                return nullptr;
            }
            if(bit_at(key, node->branch_bit)) {
                auto child = remove_node(descriptor, node->right, key, removed, context);
                if(!removed) {
                    return node;
                }
                if(!child) {
                    return node->left;
                }
                ++context.metrics.path_nodes_rebuilt;
                return make_branch(descriptor, node->branch_bit, node->left, std::move(child), context);
            }
            auto child = remove_node(descriptor, node->left, key, removed, context);
            if(!removed) {
                return node;
            }
            if(!child) {
                return node->right;
            }
            ++context.metrics.path_nodes_rebuilt;
            return make_branch(descriptor, node->branch_bit, std::move(child), node->right, context);
        }

        NodePtr apply_existing_batch(
                const ContentIdentityDescriptor & descriptor,
                const NodePtr & node,
                const std::vector<const MerkleChunkMutation *> & ordered,
                std::size_t begin,
                std::size_t end,
                BuildContext & context)
        {
            if(begin == end) {
                return node;
            }
            if(!node) {
                throw std::logic_error("Merkle batch path lost an existing node");
            }
            if(node->leaf) {
                if(end - begin != 1U
                   || node->key != merkle_coordinate_key(ordered[begin]->coordinate)) {
                    throw std::logic_error("Merkle batch leaf partition is invalid");
                }
                if(ordered[begin]->remove) {
                    return nullptr;
                }
                return make_leaf(
                        descriptor,
                        ordered[begin]->coordinate,
                        ordered[begin]->cells,
                        context);
            }

            std::size_t split = begin;
            while(split < end
                  && !bit_at(
                             merkle_coordinate_key(ordered[split]->coordinate),
                             node->branch_bit)) {
                ++split;
            }
            const auto left = apply_existing_batch(
                    descriptor,
                    node->left,
                    ordered,
                    begin,
                    split,
                    context);
            const auto right = apply_existing_batch(
                    descriptor,
                    node->right,
                    ordered,
                    split,
                    end,
                    context);
            if(!left) {
                return right;
            }
            if(!right) {
                return left;
            }
            ++context.metrics.path_nodes_rebuilt;
            return make_branch(
                    descriptor,
                    node->branch_bit,
                    left,
                    right,
                    context);
        }

        void collect_metrics(
                const NodePtr & node,
                MerkleTreeMetrics & metrics)
        {
            if(!node) {
                return;
            }
            ++metrics.node_count;
            metrics.owned_bytes += sizeof(Node);
            if(node->leaf) {
                ++metrics.leaf_count;
                metrics.total_cell_count += node->cell_count;
                return;
            }
            ++metrics.branch_count;
            collect_metrics(node->left, metrics);
            collect_metrics(node->right, metrics);
        }

        bool validate_mutation(
                const MerkleChunkMutation & mutation,
                std::uint32_t chunk_edge,
                std::string & diagnostic)
        {
            if(mutation.remove) {
                if(!mutation.cells.empty()) {
                    diagnostic = "Merkle remove mutation must not carry cells";
                    return false;
                }
                return true;
            }
            if(mutation.cells.empty()) {
                diagnostic = "Merkle upsert mutation must carry a non-empty chunk";
                return false;
            }
            if(!validate_cells(mutation.cells, diagnostic)) {
                return false;
            }
            for(const auto & cell : mutation.cells) {
                const auto address = locate_chunk(cell.index, chunk_edge);
                if(!address || address.address.chunk != mutation.coordinate) {
                    diagnostic = "Merkle mutation cell is outside its chunk coordinate";
                    return false;
                }
            }
            return true;
        }

        bool validate_chunk(
                const ChunkCoordinate & coordinate,
                const std::vector<CanonicalCell> & cells,
                std::uint32_t chunk_edge,
                std::string & diagnostic)
        {
            if(cells.empty()) {
                diagnostic = "Merkle chunk must not be empty";
                return false;
            }
            if(!validate_cells(cells, diagnostic)) {
                return false;
            }
            for(const auto & cell : cells) {
                const auto address = locate_chunk(cell.index, chunk_edge);
                if(!address || address.address.chunk != coordinate) {
                    diagnostic = "Merkle cell is outside its chunk coordinate";
                    return false;
                }
            }
            return true;
        }

        MerkleTreeMetrics metrics_for_tree(
                const NodePtr & root,
                const BuildContext & context)
        {
            MerkleTreeMetrics metrics = context.metrics;
            metrics.allocated_nodes = context.metrics.allocated_nodes;
            metrics.leaf_hashes = context.metrics.leaf_hashes;
            metrics.branch_hashes = context.metrics.branch_hashes;
            metrics.path_nodes_rebuilt = context.metrics.path_nodes_rebuilt;
            collect_metrics(root, metrics);
            return metrics;
        }

        MerkleTreeMetrics metrics_for_candidate(
                std::size_t leaf_count,
                std::size_t total_cell_count,
                const BuildContext & context)
        {
            if(leaf_count > std::numeric_limits<std::size_t>::max() / 2U + 1U) {
                throw std::overflow_error("Merkle node count overflows");
            }
            MerkleTreeMetrics metrics = context.metrics;
            metrics.leaf_count = leaf_count;
            metrics.branch_count = leaf_count == 0U ? 0U : leaf_count - 1U;
            metrics.node_count = leaf_count == 0U ? 0U : leaf_count * 2U - 1U;
            metrics.total_cell_count = total_cell_count;
            if(metrics.node_count > std::numeric_limits<std::size_t>::max() / sizeof(Node)) {
                throw std::overflow_error("Merkle owned byte count overflows");
            }
            metrics.owned_bytes = metrics.node_count * sizeof(Node);
            return metrics;
        }

    }// namespace

    struct MerklePatriciaTree::State {
        ContentIdentityDescriptor descriptor;
        SourceIdentity source;
        Hash256 geometry_fingerprint {};
        NodePtr root;
        Hash256 trie_root {};
        Hash256 content_root {};
        MerkleTreeMetrics metrics;
    };

    bool MerkleCoordinateKey::operator==(
            const MerkleCoordinateKey & other) const noexcept
    {
        return bytes == other.bytes;
    }

    bool MerkleCoordinateKey::operator!=(
            const MerkleCoordinateKey & other) const noexcept
    {
        return !(*this == other);
    }

    bool MerkleCoordinateKey::operator<(
            const MerkleCoordinateKey & other) const noexcept
    {
        return bytes < other.bytes;
    }

    MerkleCoordinateKey merkle_coordinate_key(const ChunkCoordinate & coordinate) noexcept
    {
        MerkleCoordinateKey result;
        write_key_axis(result.bytes, 0U, coordinate.x);
        write_key_axis(result.bytes, 8U, coordinate.y);
        write_key_axis(result.bytes, 16U, coordinate.z);
        return result;
    }

    MerkleCoordinateKey merkle_coordinate_key(const VoxelIndex & coordinate) noexcept
    {
        return merkle_coordinate_key(ChunkCoordinate {coordinate.x, coordinate.y, coordinate.z});
    }

    MerklePatriciaTree::MerklePatriciaTree(std::shared_ptr<const State> state)
            : state_(std::move(state))
    {
    }

    MerkleTreeResult MerklePatriciaTree::full_rebuild(
            const SourceIdentity & source,
            const Hash256 & geometry_fingerprint,
            const std::vector<CanonicalCell> & cells,
            ContentIdentityDescriptor descriptor)
    {
        MerkleTreeResult result;
        if(!descriptor.valid()) {
            result.diagnostic = "Merkle content identity descriptor is invalid";
            return result;
        }

        try {
            std::vector<ChunkLeaf> leaves;
            std::size_t total_cell_count = 0U;
            const auto visit = [&cells](const auto & consumer) {
                for(const auto & cell : cells) {
                    consumer(cell);
                }
            };
            if(!collect_chunks(
                       visit,
                       descriptor.chunk_edge,
                       leaves,
                       total_cell_count,
                       result.diagnostic)) {
                return result;
            }
            BuildContext context;
            NodePtr root;
            if(!leaves.empty()) {
                std::vector<NodePtr> leaf_nodes;
                leaf_nodes.reserve(leaves.size());
                for(const auto & leaf : leaves) {
                    leaf_nodes.push_back(make_leaf(
                            descriptor,
                            leaf.coordinate,
                            leaf.cells,
                            context));
                }
                root = build_range(
                        descriptor,
                        leaf_nodes,
                        0U,
                        leaf_nodes.size(),
                        context);
            }
            auto state = std::make_shared<State>();
            state->descriptor = descriptor;
            state->source = source;
            state->geometry_fingerprint = geometry_fingerprint;
            state->root = std::move(root);
            state->trie_root = state->root ? state->root->hash : hash_empty(descriptor);
            const auto content_begin = Clock::now();
            state->content_root = hash_content(
                    descriptor,
                    source,
                    geometry_fingerprint,
                    total_cell_count,
                    state->trie_root);
            const auto content_end = Clock::now();
            context.metrics.content_hash_ns += elapsed_ns(content_begin, content_end);
            state->metrics = metrics_for_tree(state->root, context);
            result.success = true;
            result.metrics = state->metrics;
            result.tree = std::shared_ptr<MerklePatriciaTree>(
                    new MerklePatriciaTree(std::move(state)));
            return result;
        }
        catch(const std::exception & error) {
            result.diagnostic = error.what();
            return result;
        }
    }

    MerkleTreeResult MerklePatriciaTree::full_rebuild(
            const SourceIdentity & source,
            const Hash256 & geometry_fingerprint,
            const CanonicalCellView & cells,
            ContentIdentityDescriptor descriptor)
    {
        MerkleTreeResult result;
        if(!descriptor.valid()) {
            result.diagnostic = "Merkle content identity descriptor is invalid";
            return result;
        }
        try {
            std::vector<ChunkLeaf> leaves;
            std::size_t total_cell_count = 0U;
            const auto visit = [&cells](const auto & consumer) { cells.for_each(consumer); };
            if(!collect_chunks(
                       visit,
                       descriptor.chunk_edge,
                       leaves,
                       total_cell_count,
                       result.diagnostic)) {
                return result;
            }
            BuildContext context;
            NodePtr root;
            if(!leaves.empty()) {
                std::vector<NodePtr> leaf_nodes;
                leaf_nodes.reserve(leaves.size());
                for(const auto & leaf : leaves) {
                    leaf_nodes.push_back(make_leaf(
                            descriptor,
                            leaf.coordinate,
                            leaf.cells,
                            context));
                }
                root = build_range(
                        descriptor,
                        leaf_nodes,
                        0U,
                        leaf_nodes.size(),
                        context);
            }
            auto state = std::make_shared<State>();
            state->descriptor = descriptor;
            state->source = source;
            state->geometry_fingerprint = geometry_fingerprint;
            state->root = std::move(root);
            state->trie_root = state->root ? state->root->hash : hash_empty(descriptor);
            const auto content_begin = Clock::now();
            state->content_root = hash_content(
                    descriptor,
                    source,
                    geometry_fingerprint,
                    total_cell_count,
                    state->trie_root);
            const auto content_end = Clock::now();
            context.metrics.content_hash_ns += elapsed_ns(content_begin, content_end);
            state->metrics = metrics_for_tree(state->root, context);
            result.success = true;
            result.metrics = state->metrics;
            result.tree = std::shared_ptr<MerklePatriciaTree>(
                    new MerklePatriciaTree(std::move(state)));
            return result;
        }
        catch(const std::exception & error) {
            result.diagnostic = error.what();
            return result;
        }
    }

    MerkleTreeResult MerklePatriciaTree::full_rebuild(
            const SourceIdentity & source,
            const Hash256 & geometry_fingerprint,
            const CellSnapshotStore & store,
            ContentIdentityDescriptor descriptor)
    {
        MerkleTreeResult result;
        if(!descriptor.valid()) {
            result.diagnostic = "Merkle content identity descriptor is invalid";
            return result;
        }
        try {
            BuildContext context;
            std::vector<NodePtr> leaf_nodes;
            std::size_t total_cell_count = 0U;
            bool valid = true;
            store.for_each_chunk([&](
                                         const ChunkCoordinate & coordinate,
                                         const std::vector<CanonicalCell> & cells) {
                if(!valid) {
                    return;
                }
                if(!validate_chunk(
                           coordinate,
                           cells,
                           descriptor.chunk_edge,
                           result.diagnostic)) {
                    valid = false;
                    return;
                }
                if(cells.size() > std::numeric_limits<std::size_t>::max() - total_cell_count) {
                    result.diagnostic = "Merkle cell count overflows";
                    valid = false;
                    return;
                }
                leaf_nodes.push_back(make_leaf(descriptor, coordinate, cells, context));
                total_cell_count += cells.size();
            });
            if(!valid) {
                return result;
            }
            NodePtr root;
            if(!leaf_nodes.empty()) {
                root = build_range(
                        descriptor,
                        leaf_nodes,
                        0U,
                        leaf_nodes.size(),
                        context);
            }
            auto state = std::make_shared<State>();
            state->descriptor = descriptor;
            state->source = source;
            state->geometry_fingerprint = geometry_fingerprint;
            state->root = std::move(root);
            state->trie_root = state->root ? state->root->hash : hash_empty(descriptor);
            const auto content_begin = Clock::now();
            state->content_root = hash_content(
                    descriptor,
                    source,
                    geometry_fingerprint,
                    total_cell_count,
                    state->trie_root);
            const auto content_end = Clock::now();
            context.metrics.content_hash_ns += elapsed_ns(content_begin, content_end);
            state->metrics = metrics_for_tree(state->root, context);
            result.success = true;
            result.metrics = state->metrics;
            result.tree = std::shared_ptr<MerklePatriciaTree>(
                    new MerklePatriciaTree(std::move(state)));
            return result;
        }
        catch(const std::exception & error) {
            result.diagnostic = error.what();
            return result;
        }
    }

    MerkleTreeResult MerklePatriciaTree::apply(
            const std::vector<MerkleChunkMutation> & mutations) const
    {
        MerkleTreeResult result;
        if(!state_) {
            result.diagnostic = "Merkle tree has no state";
            return result;
        }

        std::vector<const MerkleChunkMutation *> ordered;
        ordered.reserve(mutations.size());
        for(const auto & mutation : mutations) {
            ordered.push_back(&mutation);
        }
        std::sort(
                ordered.begin(),
                ordered.end(),
                [](const MerkleChunkMutation * left, const MerkleChunkMutation * right) {
                    return left->coordinate < right->coordinate;
                });
        for(std::size_t index = 0U; index < ordered.size(); ++index) {
            if(index > 0U && ordered[index - 1U]->coordinate == ordered[index]->coordinate) {
                result.diagnostic = "Merkle mutation list contains duplicate chunk coordinates";
                return result;
            }
            if(!validate_mutation(
                       *ordered[index],
                       state_->descriptor.chunk_edge,
                       result.diagnostic)) {
                return result;
            }
        }

        try {
            NodePtr root = state_->root;
            std::size_t total_cell_count = state_->metrics.total_cell_count;
            std::size_t leaf_count = state_->metrics.leaf_count;
            BuildContext context;
            bool all_existing = true;
            for(const auto * mutation : ordered) {
                if(!find_leaf(root, merkle_coordinate_key(mutation->coordinate))) {
                    all_existing = false;
                    break;
                }
            }

            for(const auto * mutation : ordered) {
                const auto key = merkle_coordinate_key(mutation->coordinate);
                const auto * old_leaf = find_leaf(root, key);
                if(mutation->remove) {
                    if(!old_leaf) {
                        result.diagnostic = "Merkle remove mutation targets an unknown chunk";
                        return result;
                    }
                    total_cell_count -= old_leaf->cell_count;
                    --leaf_count;
                    continue;
                }

                if(old_leaf) {
                    total_cell_count -= old_leaf->cell_count;
                }
                else {
                    ++leaf_count;
                }
                if(mutation->cells.size() > std::numeric_limits<std::size_t>::max() - total_cell_count) {
                    result.diagnostic = "Merkle cell count overflows";
                    return result;
                }
                total_cell_count += mutation->cells.size();
            }

            if(all_existing) {
                root = apply_existing_batch(
                        state_->descriptor,
                        root,
                        ordered,
                        0U,
                        ordered.size(),
                        context);
            }
            else {
                for(const auto * mutation : ordered) {
                    const auto key = merkle_coordinate_key(mutation->coordinate);
                    if(mutation->remove) {
                        bool removed = false;
                        root = remove_node(state_->descriptor, root, key, removed, context);
                        if(!removed) {
                            result.diagnostic = "Merkle remove mutation targets an unknown chunk";
                            return result;
                        }
                        continue;
                    }
                    auto leaf = make_leaf(
                            state_->descriptor,
                            mutation->coordinate,
                            mutation->cells,
                            context);
                    root = insert_node(state_->descriptor, root, std::move(leaf), context);
                }
            }

            auto candidate = std::make_shared<State>();
            candidate->descriptor = state_->descriptor;
            candidate->source = state_->source;
            candidate->geometry_fingerprint = state_->geometry_fingerprint;
            candidate->root = std::move(root);
            candidate->trie_root = candidate->root
                                           ? candidate->root->hash
                                           : hash_empty(candidate->descriptor);
            const auto content_begin = Clock::now();
            candidate->content_root = hash_content(
                    candidate->descriptor,
                    candidate->source,
                    candidate->geometry_fingerprint,
                    total_cell_count,
                    candidate->trie_root);
            const auto content_end = Clock::now();
            context.metrics.content_hash_ns += elapsed_ns(content_begin, content_end);
            candidate->metrics = metrics_for_candidate(
                    leaf_count,
                    total_cell_count,
                    context);
            result.success = true;
            result.metrics = candidate->metrics;
            result.tree = std::shared_ptr<MerklePatriciaTree>(
                    new MerklePatriciaTree(std::move(candidate)));
            return result;
        }
        catch(const std::exception & error) {
            result.diagnostic = error.what();
            return result;
        }
    }

    const ContentIdentityDescriptor & MerklePatriciaTree::descriptor() const noexcept
    {
        return state_->descriptor;
    }

    const SourceIdentity & MerklePatriciaTree::source() const noexcept
    {
        return state_->source;
    }

    const Hash256 & MerklePatriciaTree::geometry_fingerprint() const noexcept
    {
        return state_->geometry_fingerprint;
    }

    const Hash256 & MerklePatriciaTree::trie_root() const noexcept
    {
        return state_->trie_root;
    }

    Hash256 MerklePatriciaTree::content_root() const noexcept
    {
        return state_->content_root;
    }

    VersionedContentDigest MerklePatriciaTree::versioned_digest() const noexcept
    {
        return {state_->descriptor, state_->content_root};
    }

    const MerkleTreeMetrics & MerklePatriciaTree::metrics() const noexcept
    {
        return state_->metrics;
    }

    std::size_t MerklePatriciaTree::leaf_count() const noexcept
    {
        return state_->metrics.leaf_count;
    }

    std::size_t MerklePatriciaTree::total_cell_count() const noexcept
    {
        return state_->metrics.total_cell_count;
    }

    bool MerklePatriciaTree::empty() const noexcept
    {
        return state_->root == nullptr;
    }

    bool MerklePatriciaTree::estimate_node_bytes(
            std::size_t node_count,
            std::size_t & bytes) noexcept
    {
        if(node_count > std::numeric_limits<std::size_t>::max() / sizeof(Node)) {
            return false;
        }
        bytes = node_count * sizeof(Node);
        return true;
    }

}// namespace PerceptionMapUpdate
