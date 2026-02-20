#![allow(dead_code)]

use anyhow::Result;
use crazyflie_lib::{Crazyflie, TocCache};
use crazyflie_link::LinkContext;
use serde::Deserialize;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, OnceLock, RwLock};

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
    site_path.push("../..");
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

/// In-memory TOC cache
#[derive(Clone)]
pub struct InMemoryTocCache {
    cache: Arc<RwLock<HashMap<Vec<u8>, String>>>,
}

impl InMemoryTocCache {
    pub fn new() -> Self {
        InMemoryTocCache {
            cache: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

impl TocCache for InMemoryTocCache {
    fn get_toc(&self, key: &[u8]) -> Option<String> {
        self.cache.read().ok()?.get(key).cloned()
    }

    fn store_toc(&self, key: &[u8], toc: &str) {
        if let Ok(mut cache) = self.cache.write() {
            cache.insert(key.to_vec(), toc.to_string());
        }
    }
}

static TOC_CACHE: OnceLock<InMemoryTocCache> = OnceLock::new();

/// Connect to a Crazyflie using a process-wide shared in-memory TOC cache.
/// The first connection per drone fetches the TOC over radio; all subsequent
/// connections within the same test binary hit the cache.
pub async fn connect_crazyflie(ctx: &LinkContext, uri: &str) -> Result<Crazyflie> {
    let cache = TOC_CACHE.get_or_init(InMemoryTocCache::new);
    Ok(Crazyflie::connect_from_uri(ctx, uri, cache.clone()).await?)
}

pub fn init_logging() {
    let _ = env_logger::builder().is_test(true).try_init();
}

/// Logging requirements from requirements/logging.toml
#[derive(Debug, Deserialize)]
pub struct RequirementFile {
    pub requirement: RequirementGroup,
}

#[derive(Debug, Deserialize)]
pub struct RequirementGroup {
    pub logging: LoggingRequirements,
}

#[derive(Debug, Deserialize)]
pub struct LoggingRequirements {
    pub basic: LoggingBasic,
    pub variables: LoggingVariables,
    pub blocks: LoggingBlocks,
    pub rate: LoggingRate,
}

#[derive(Debug, Deserialize)]
pub struct LoggingBasic {
    pub max_rate: u32,
}

#[derive(Debug, Deserialize)]
pub struct LoggingVariables {
    pub max: u32,
}

#[derive(Debug, Deserialize)]
pub struct LoggingBlocks {
    pub max: u32,
    pub max_payload: u32,
}

#[derive(Debug, Deserialize)]
pub struct LoggingRate {
    pub limit_low: u32,
}

/// Load logging requirements from requirements/logging.toml
pub fn load_logging_requirements() -> Result<LoggingRequirements> {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("requirements");
    path.push("logging.toml");

    let contents = fs::read_to_string(&path)?;
    let file: RequirementFile = toml::from_str(&contents)?;

    Ok(file.requirement.logging)
}
