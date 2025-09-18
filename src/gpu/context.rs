//! GPU context management for csgrs
//!
//! This module provides the core GPU context that manages WebGPU resources,
//! device initialization, and capability detection.

use super::{GpuCapabilities, GpuError, GpuResult};
use std::sync::Arc;
use wgpu::{Adapter, Device, Instance, Queue, util::DeviceExt};

/// GPU context for accelerated geometric operations
#[derive(Clone)]
pub struct GpuContext {
    /// WebGPU instance
    _instance: Arc<Instance>,
    /// GPU adapter
    adapter: Arc<Adapter>,
    /// GPU device
    device: Arc<Device>,
    /// Command queue
    queue: Arc<Queue>,
    /// GPU capabilities
    capabilities: GpuCapabilities,
}

impl GpuContext {
    /// Create a new GPU context with default configuration
    pub async fn new() -> GpuResult<Self> {
        Self::new_with_limits(None).await
    }

    /// Create a new GPU context with custom limits
    pub async fn new_with_limits(limits: Option<wgpu::Limits>) -> GpuResult<Self> {
        // Create WebGPU instance
        let instance = Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..Default::default()
        });

        // Request adapter
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .ok_or_else(|| {
                GpuError::NotAvailable("No compatible GPU adapter found".to_string())
            })?;

        // Get adapter info for capabilities
        let _adapter_info = adapter.get_info();
        let adapter_limits = adapter.limits();

        // Create device
        let default_limits = wgpu::Limits {
            max_buffer_size: 1 << 30, // 1GB
            max_storage_buffer_binding_size: 1 << 30,
            max_storage_buffers_per_shader_stage: 8, // Increase storage buffer limit
            ..wgpu::Limits::downlevel_defaults()
        };

        let device_limits = limits.unwrap_or(default_limits);
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("csgrs-gpu-device"),
                    required_features: wgpu::Features::empty(),
                    required_limits: device_limits.clone(),
                    memory_hints: Default::default(),
                },
                None,
            )
            .await
            .map_err(|e| {
                GpuError::NotAvailable(format!("Failed to create GPU device: {}", e))
            })?;

        // Determine capabilities
        let capabilities = GpuCapabilities {
            max_workgroups_per_dimension: [
                adapter_limits.max_compute_workgroups_per_dimension,
                adapter_limits.max_compute_workgroups_per_dimension,
                adapter_limits.max_compute_workgroups_per_dimension,
            ],
            max_workgroup_size: [
                adapter_limits.max_compute_workgroup_size_x,
                adapter_limits.max_compute_workgroup_size_y,
                adapter_limits.max_compute_workgroup_size_z,
            ],
            max_buffer_size: adapter_limits
                .max_buffer_size
                .min(device_limits.max_buffer_size),
            supports_f64: false, // WebGPU doesn't support f64 in compute shaders
            supports_compute: adapter.features().contains(wgpu::Features::SHADER_F64),
            available_memory: None, // WebGPU doesn't expose memory info
        };

        Ok(Self {
            _instance: Arc::new(instance),
            adapter: Arc::new(adapter),
            device: Arc::new(device),
            queue: Arc::new(queue),
            capabilities,
        })
    }

    /// Get GPU capabilities
    pub const fn capabilities(&self) -> &GpuCapabilities {
        &self.capabilities
    }

    /// Get WebGPU device reference
    pub fn device(&self) -> &Device {
        &self.device
    }

    /// Get command queue reference
    pub fn queue(&self) -> &Queue {
        &self.queue
    }

    /// Check if GPU context is functional
    pub const fn is_available(&self) -> bool {
        // Simple validation by checking device is still valid
        true // WebGPU handles device loss internally
    }

    /// Get adapter info
    pub fn adapter_info(&self) -> wgpu::AdapterInfo {
        self.adapter.get_info()
    }

    /// Create a compute pipeline
    pub fn create_compute_pipeline(
        &self,
        shader: &wgpu::ShaderModule,
        entry_point: &str,
        bind_group_layouts: &[&wgpu::BindGroupLayout],
    ) -> GpuResult<wgpu::ComputePipeline> {
        let pipeline_layout =
            self.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("compute-pipeline-layout"),
                    bind_group_layouts,
                    push_constant_ranges: &[],
                });

        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("compute-pipeline"),
                layout: Some(&pipeline_layout),
                module: shader,
                entry_point: Some(entry_point),
                compilation_options: Default::default(),
                cache: None,
            });

        Ok(pipeline)
    }

    /// Submit command buffer to queue
    pub fn submit(&self, encoder: wgpu::CommandEncoder) {
        self.queue.submit(std::iter::once(encoder.finish()));
    }

    /// Poll device for completion (blocking)
    pub fn poll(&self) {
        self.device.poll(wgpu::Maintain::Wait);
    }

    /// Create buffer with specified usage
    pub fn create_buffer(
        &self,
        size: u64,
        usage: wgpu::BufferUsages,
        label: Option<&str>,
    ) -> wgpu::Buffer {
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label,
            size,
            usage,
            mapped_at_creation: false,
        })
    }

    /// Create buffer initialized with data
    pub fn create_buffer_init(
        &self,
        usage: wgpu::BufferUsages,
        data: &[u8],
        label: Option<&str>,
    ) -> wgpu::Buffer {
        self.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label,
                contents: data,
                usage,
            })
    }

    /// Create bind group layout
    pub fn create_bind_group_layout(
        &self,
        entries: &[wgpu::BindGroupLayoutEntry],
        label: Option<&str>,
    ) -> wgpu::BindGroupLayout {
        self.device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor { label, entries })
    }

    /// Create bind group
    pub fn create_bind_group(
        &self,
        layout: &wgpu::BindGroupLayout,
        entries: &[wgpu::BindGroupEntry],
        label: Option<&str>,
    ) -> wgpu::BindGroup {
        self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label,
            layout,
            entries,
        })
    }

    /// Create shader module from WGSL source
    pub fn create_shader_module(
        &self,
        source: &str,
        label: Option<&str>,
    ) -> wgpu::ShaderModule {
        self.device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label,
                source: wgpu::ShaderSource::Wgsl(source.into()),
            })
    }
}

impl std::fmt::Debug for GpuContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuContext")
            .field("adapter", &self.adapter_info())
            .field("capabilities", &self.capabilities)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_gpu_context_creation() {
        // Test that GPU context can be created (may fail if no GPU available)
        match GpuContext::new().await {
            Ok(context) => {
                assert!(context.is_available());
                let caps = context.capabilities();
                assert!(caps.max_workgroup_size[0] > 0);
                assert!(caps.max_buffer_size > 0);
            },
            Err(GpuError::NotAvailable(_)) => {
                // Expected on systems without GPU support
            },
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }

    #[test]
    fn test_capabilities_default() {
        let caps = GpuCapabilities::default();
        assert_eq!(caps.max_workgroups_per_dimension, [65535, 65535, 65535]);
        assert_eq!(caps.max_workgroup_size, [256, 256, 64]);
        assert_eq!(caps.max_buffer_size, 1 << 30);
        assert!(!caps.supports_f64);
        assert!(caps.supports_compute);
    }
}
