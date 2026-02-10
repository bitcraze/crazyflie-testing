#![allow(dead_code)]

use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache, TocCache};
use crazyflie_link::LinkContext;
use serde::Deserialize;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};

pub const DEFAULT_SITE: &str = "single-cf";

/// Site configuration format matching sites/*.toml
#[derive(Debug, Deserialize)]
pub struct SiteConfig {
    pub version: u32,
    #[serde(default)]
    pub rig_management: Option<String>,
    pub device: HashMap<String, DeviceConfig>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct DeviceConfig {
    pub radio: String,
    #[serde(default)]
    pub bootloader_radio: Option<String>,
    #[serde(default)]
    pub decks: Vec<String>,
    #[serde(default = "default_platform")]
    pub platform: String,
    #[serde(default)]
    pub usb_power_control: Option<String>,
    #[serde(default)]
    pub rig_management_addr: Option<String>,
    #[serde(default)]
    pub properties: Vec<String>,
}

fn default_platform() -> String {
    "cf2".to_string()
}

/// Load site configuration from sites/ directory.
/// Uses CRAZY_SITE env var, defaulting to "single-cf" (matching Python's conftest.py).
pub fn load_site_config() -> Result<SiteConfig> {
    let site_name = env::var("CRAZY_SITE")
        .unwrap_or_else(|_| DEFAULT_SITE.to_string());

    let mut site_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    site_path.push("sites");
    site_path.push(format!("{}.toml", site_name));

    let contents = fs::read_to_string(&site_path)?;
    let config: SiteConfig = toml::from_str(&contents)?;

    Ok(config)
}

/// Get devices from site config, filtered by CRAZY_DEVICE env var.
/// CRAZY_DEVICE accepts comma-separated device names (matching Python's conftest.py).
pub fn get_devices(config: &SiteConfig) -> Vec<(String, DeviceConfig)> {
    let device_filter: Option<Vec<String>> = env::var("CRAZY_DEVICE")
        .ok()
        .filter(|v| !v.is_empty())
        .map(|v| v.split(',').map(|s| s.trim().to_string()).collect());

    config
        .device
        .iter()
        .filter(|(name, _)| match &device_filter {
            Some(names) => names.contains(name),
            None => true,
        })
        .map(|(name, device)| (name.clone(), device.clone()))
        .collect()
}

/// Simple file-based TOC cache with in-memory layer
#[derive(Clone)]
pub struct FileTocCache {
    cache_dir: PathBuf,
    memory_cache: Arc<RwLock<HashMap<u32, String>>>,
}

impl FileTocCache {
    pub fn new(cache_dir: PathBuf) -> Self {
        fs::create_dir_all(&cache_dir).ok();
        FileTocCache {
            cache_dir,
            memory_cache: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    fn cache_file_path(&self, crc32: u32) -> PathBuf {
        self.cache_dir.join(format!("toc_{:08x}.json", crc32))
    }
}

impl TocCache for FileTocCache {
    fn get_toc(&self, crc32: u32) -> Option<String> {
        // Try memory cache first
        if let Ok(lock) = self.memory_cache.read() {
            if let Some(toc) = lock.get(&crc32) {
                return Some(toc.clone());
            }
        }

        // Try file cache
        let path = self.cache_file_path(crc32);
        if let Ok(toc) = fs::read_to_string(&path) {
            // Store in memory cache for next time
            if let Ok(mut lock) = self.memory_cache.write() {
                lock.insert(crc32, toc.clone());
            }
            return Some(toc);
        }

        None
    }

    fn store_toc(&self, crc32: u32, toc: &str) {
        // Store in memory cache
        if let Ok(mut lock) = self.memory_cache.write() {
            lock.insert(crc32, toc.to_string());
        }

        // Store in file cache
        let path = self.cache_file_path(crc32);
        fs::write(path, toc).ok();
    }
}

pub fn get_cache_dir() -> PathBuf {
    env::temp_dir().join("crazyflie_rust_test_cache")
}

pub fn use_toc_cache() -> bool {
    env::var("USE_TOC_CACHE")
        .map(|v| v == "1" || v.to_lowercase() == "true")
        .unwrap_or(false)
}

/// Connect to a Crazyflie, optionally using TOC cache based on USE_TOC_CACHE env var
pub async fn connect_crazyflie(ctx: &LinkContext, uri: &str) -> Result<Crazyflie> {
    if use_toc_cache() {
        let cache = FileTocCache::new(get_cache_dir());
        Ok(Crazyflie::connect_from_uri(ctx, uri, cache).await?)
    } else {
        Ok(Crazyflie::connect_from_uri(ctx, uri, NoTocCache).await?)
    }
}

pub fn init_logging() {
    let _ = env_logger::builder().is_test(true).try_init();
}
