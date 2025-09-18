//! GPU-accelerated geometric operations for csgrs
//!
//! This module provides high-performance GPU acceleration for CSG boolean operations,
//! mesh processing, and geometric algorithms using WebGPU (wgpu) compute shaders.
//!
//! # Current Implementation Status
//!
//! **PHASE 1 COMPLETE**: Core WGPU infrastructure and shader framework established
//! - ✅ GPU context management with WebGPU initialization
//! - ✅ Buffer management for mesh data transfer to/from GPU
//! - ✅ Compute pipeline creation with WGSL shader compilation
//! - ✅ Bind group layouts and resource management
//! - ✅ CPU fallback system for reliability
//!
//! **PHASE 2 COMPLETE**: GPU compute shader framework implemented
//! - ✅ WGSL shader source code for boolean operations
//! - ✅ Pipeline creation and dispatch framework
//! - ✅ GPU buffer data transfer and result extraction
//! - ✅ GPU computation execution framework (CPU fallback active)
//!
//! **PHASE 3 COMPLETE**: Sparse voxel GPU acceleration framework established
//! - ✅ GPU buffer layouts for sparse voxel octrees (Sprint 80)
//! - ✅ WGSL compute shader framework for voxel CSG operations (Sprint 80)
//! - ✅ CPU fallback implementation with GPU operation traits (Sprint 80)
//! - 📋 Full GPU shader implementation (Future enhancement)
//!
//! # Architecture Overview
//!
//! The GPU acceleration system consists of:
//! - **Compute Shaders**: WGSL shaders for parallel geometric operations
//! - **Memory Management**: Efficient GPU buffer allocation and data transfer
//! - **Operation Pipelines**: Pre-compiled compute pipelines for common operations
//! - **Fallback System**: Automatic fallback to CPU when GPU unavailable
//!
//! # Supported Operations
//!
//! - Boolean operations (union, difference, intersection, XOR) on indexed meshes
//! - Sparse voxel octree boolean operations (framework established, CPU fallback active)
//! - Mesh vertex transformations and normal calculations
//! - Geometric primitive generation on GPU
//! - Batch processing for multiple operations
//!
//! # Performance Characteristics (Target)
//!
//! - **Throughput**: 10-50x speedup for large mesh operations (currently CPU fallback)
//! - **Memory**: Efficient GPU memory usage with streaming
//! - **Scalability**: Handles millions of vertices efficiently
//! - **Precision**: Full f32/f64 precision support
//!
//! # Usage
//!
//! ```rust,ignore
//! use csgrs::gpu::GpuContext;
//! use csgrs::indexed_mesh::IndexedMesh;
//!
//! // Initialize GPU context
//! let gpu = GpuContext::new().await?;
//!
//! // Perform GPU-accelerated boolean operations
//! let result = gpu.union(&mesh1, &mesh2).await?;
//! ```

pub mod buffers;
pub mod context;
pub mod operations;
pub mod pipeline;
pub mod shaders;

pub use context::GpuContext;
pub use operations::GpuBooleanOps;
pub use operations::should_use_gpu;
pub use pipeline::ComputePipelines;

/// Error types for GPU operations
#[derive(Debug, thiserror::Error)]
pub enum GpuError {
    #[error("GPU not available: {0}")]
    NotAvailable(String),

    #[error("Shader compilation failed: {0}")]
    ShaderCompilation(String),

    #[error("Pipeline creation failed: {0}")]
    PipelineCreation(String),

    #[error("Buffer operation failed: {0}")]
    BufferError(String),

    #[error("Compute dispatch failed: {0}")]
    DispatchError(String),

    #[error("Memory allocation failed: {0}")]
    OutOfMemory(String),

    #[error("Data transfer failed: {0}")]
    TransferError(String),

    #[error("Operation timeout: {0}")]
    Timeout(String),

    #[error("Unsupported operation: {0}")]
    Unsupported(String),
}

/// Result type for GPU operations
pub type GpuResult<T> = Result<T, GpuError>;

/// GPU vertex representation for compute shaders
#[derive(Debug, Clone, Copy, Default, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct GpuVertex {
    /// Position coordinates (x, y, z)
    pub position: [f32; 3],
    /// Normal vector (x, y, z)
    pub normal: [f32; 3],
    /// Metadata value
    pub metadata: u32,
}

/// GPU voxel node representation for compute shaders
#[derive(Debug, Clone, Copy, Default, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct GpuVoxelNode {
    /// Node type and flags (0=internal, 1=leaf, bit 1=occupied for leaves)
    pub node_type: u32,
    /// Child node indices (-1 if no child, for internal nodes)
    pub children: [i32; 8],
    /// Node depth in octree
    pub depth: u32,
    /// Node origin coordinates (x, y, z)
    pub origin: [f32; 3],
    /// Node size (uniform in all dimensions)
    pub size: f32,
    /// Metadata index (for leaf nodes)
    pub metadata_index: i32,
}

/// GPU voxel octree header
#[derive(Debug, Clone, Copy, Default, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct GpuVoxelHeader {
    /// Total number of nodes in the octree
    pub node_count: u32,
    /// Maximum depth of the octree
    pub max_depth: u32,
    /// Root node index (always 0)
    pub root_index: u32,
    /// Number of occupied leaves
    pub occupied_leaves: u32,
    /// Origin of the octree (x, y, z)
    pub origin: [f32; 3],
    /// Size of root volume
    pub root_size: f32,
}

/// GPU capabilities and limits
#[derive(Debug, Clone)]
pub struct GpuCapabilities {
    /// Maximum number of workgroups per dimension
    pub max_workgroups_per_dimension: [u32; 3],
    /// Maximum workgroup size
    pub max_workgroup_size: [u32; 3],
    /// Maximum buffer size in bytes
    pub max_buffer_size: u64,
    /// Supports f64 precision
    pub supports_f64: bool,
    /// Compute shader support
    pub supports_compute: bool,
    /// Available VRAM in bytes
    pub available_memory: Option<u64>,
}

impl Default for GpuCapabilities {
    fn default() -> Self {
        Self {
            max_workgroups_per_dimension: [65535, 65535, 65535],
            max_workgroup_size: [256, 256, 64],
            max_buffer_size: 1 << 30, // 1GB
            supports_f64: false,
            supports_compute: true,
            available_memory: None,
        }
    }
}
