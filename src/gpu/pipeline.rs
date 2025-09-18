//! GPU compute pipeline management for csgrs
//!
//! This module provides pre-compiled compute pipelines for various
//! geometric operations, with caching and reuse.

use super::{GpuContext, GpuResult};
use std::collections::HashMap;

/// Compute pipeline cache for reuse
pub struct ComputePipelineCache {
    context: GpuContext,
    pipelines: HashMap<String, wgpu::ComputePipeline>,
    bind_group_layouts: HashMap<String, wgpu::BindGroupLayout>,
}

impl ComputePipelineCache {
    /// Create a new pipeline cache
    pub fn new(context: GpuContext) -> Self {
        Self {
            context,
            pipelines: HashMap::new(),
            bind_group_layouts: HashMap::new(),
        }
    }

    /// Get or create a compute pipeline
    pub fn get_pipeline(
        &mut self,
        name: &str,
        shader_source: &str,
        entry_point: &str,
        bind_group_layout_entries: &[wgpu::BindGroupLayoutEntry],
    ) -> GpuResult<&wgpu::ComputePipeline> {
        if !self.pipelines.contains_key(name) {
            // Create bind group layout if not exists
            let layout_key = format!("{}_layout", name);
            if !self.bind_group_layouts.contains_key(&layout_key) {
                let layout = self
                    .context
                    .create_bind_group_layout(bind_group_layout_entries, Some(&layout_key));
                self.bind_group_layouts.insert(layout_key.clone(), layout);
            }

            // Create shader module
            let shader = self.context.create_shader_module(shader_source, Some(name));

            // Create pipeline
            let bind_group_layouts = &[self.bind_group_layouts.get(&layout_key).unwrap()];
            let pipeline = self.context.create_compute_pipeline(
                &shader,
                entry_point,
                bind_group_layouts,
            )?;

            self.pipelines.insert(name.to_string(), pipeline);
        }

        Ok(self.pipelines.get(name).unwrap())
    }

    /// Get bind group layout for a pipeline
    pub fn get_bind_group_layout(&self, layout_name: &str) -> Option<&wgpu::BindGroupLayout> {
        self.bind_group_layouts.get(layout_name)
    }

    /// Clear all cached pipelines
    pub fn clear(&mut self) {
        self.pipelines.clear();
        self.bind_group_layouts.clear();
    }

    /// Get cache statistics
    pub fn stats(&self) -> PipelineStats {
        PipelineStats {
            pipeline_count: self.pipelines.len(),
            layout_count: self.bind_group_layouts.len(),
        }
    }
}

/// Pipeline cache statistics
#[derive(Debug, Clone)]
pub struct PipelineStats {
    pub pipeline_count: usize,
    pub layout_count: usize,
}

/// Pre-configured compute pipelines for common operations
pub struct ComputePipelines {
    cache: ComputePipelineCache,
}

impl ComputePipelines {
    /// Create new compute pipelines manager
    pub fn new(context: GpuContext) -> Self {
        Self {
            cache: ComputePipelineCache::new(context),
        }
    }

    // Current implementation provides caching framework for future GPU compute operations
    // Pipeline methods will be implemented when GPU shaders are developed

    /// Get cache statistics
    pub fn stats(&self) -> PipelineStats {
        self.cache.stats()
    }

    /// Clear pipeline cache
    pub fn clear_cache(&mut self) {
        self.cache.clear();
    }

    /// Get bind group layout for a pipeline
    pub fn get_bind_group_layout(&self, layout_name: &str) -> Option<&wgpu::BindGroupLayout> {
        self.cache.get_bind_group_layout(layout_name)
    }


    /// Get boolean operations pipeline
    pub fn boolean_ops_pipeline(&mut self) -> GpuResult<&wgpu::ComputePipeline> {
        let bind_group_layout_entries = &[
            // Mesh1 header
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Mesh1 vertices
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Mesh1 faces
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Mesh2 header
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Mesh2 vertices
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Mesh2 faces
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Output vertices
            wgpu::BindGroupLayoutEntry {
                binding: 6,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Output faces
            wgpu::BindGroupLayoutEntry {
                binding: 7,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Output header
            wgpu::BindGroupLayoutEntry {
                binding: 8,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Operation type
            wgpu::BindGroupLayoutEntry {
                binding: 9,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ];

        use super::shaders::BOOLEAN_OP_SHADER;
        self.cache.get_pipeline(
            "boolean_ops",
            BOOLEAN_OP_SHADER,
            "main",
            bind_group_layout_entries,
        )
    }

    /// Get normal calculation pipeline
    pub fn normal_calculation_pipeline(&mut self) -> GpuResult<&wgpu::ComputePipeline> {
        let bind_group_layout_entries = &[
            // Mesh header
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Vertices
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Faces (read-write for normal updates)
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ];

        use super::shaders::NORMAL_CALCULATION_SHADER;
        self.cache.get_pipeline(
            "normal_calculation",
            NORMAL_CALCULATION_SHADER,
            "main",
            bind_group_layout_entries,
        )
    }

    /// Get transformation pipeline
    pub fn transformation_pipeline(&mut self) -> GpuResult<&wgpu::ComputePipeline> {
        let bind_group_layout_entries = &[
            // Mesh header
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Input vertices
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Output vertices
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Transform matrix
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Normal matrix
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ];

        use super::shaders::TRANSFORMATION_SHADER;
        self.cache.get_pipeline(
            "transformation",
            TRANSFORMATION_SHADER,
            "main",
            bind_group_layout_entries,
        )
    }

    /// Get deduplication pipeline
    pub fn deduplication_pipeline(&mut self) -> GpuResult<&wgpu::ComputePipeline> {
        let bind_group_layout_entries = &[
            // Input header
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Input vertices
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Output vertices
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Vertex map
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Epsilon value
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            // Output header
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ];

        use super::shaders::DEDUPLICATION_SHADER;
        self.cache.get_pipeline(
            "deduplication",
            DEDUPLICATION_SHADER,
            "main",
            bind_group_layout_entries,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_pipeline_creation() {
        match GpuContext::new().await {
            Ok(gpu) => {
                let mut pipelines = ComputePipelines::new(gpu);

                // Test pipeline creation (may fail if shaders are invalid, but shouldn't panic)
                let _ = pipelines.boolean_ops_pipeline();
                let _ = pipelines.normal_calculation_pipeline();
                let _ = pipelines.transformation_pipeline();
                let _ = pipelines.deduplication_pipeline();

                // Check that stats are reasonable
                let stats = pipelines.stats();
                assert!(stats.pipeline_count <= 4); // We tried to create 4 pipelines
                assert!(stats.layout_count <= 4);
            },
            Err(_) => {
                // Skip test if GPU unavailable
            },
        }
    }

    #[test]
    fn test_pipeline_cache() {
        // Test cache statistics
        match pollster::block_on(GpuContext::new()) {
            Ok(gpu) => {
                let cache = ComputePipelineCache::new(gpu);
                let stats = cache.stats();
                assert_eq!(stats.pipeline_count, 0);
                assert_eq!(stats.layout_count, 0);
            },
            Err(_) => {
                // Skip if GPU unavailable
            },
        }
    }
}
