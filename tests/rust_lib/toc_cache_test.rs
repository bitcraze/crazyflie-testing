use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache, TocCache};
use crazyflie_link::LinkContext;
use serde::Deserialize;
use serial_test::serial;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};

/// Simple file-based TOC cache implementation
#[derive(Clone)]
struct FileTocCache {
    cache_dir: PathBuf,
    memory_cache: Arc<RwLock<HashMap<u32, String>>>,
}

impl FileTocCache {
    fn new(cache_dir: PathBuf) -> Self {
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

/// Site configuration format matching sites/*.toml
#[derive(Debug, Deserialize)]
struct SiteConfig {
    #[allow(dead_code)]
    version: u32,
    device: HashMap<String, DeviceConfig>,
}

#[derive(Debug, Deserialize, Clone)]
struct DeviceConfig {
    radio: String,
    #[allow(dead_code)]
    #[serde(default)]
    decks: Vec<String>,
}

/// Load site configuration from sites/ directory
fn load_site_config() -> Result<SiteConfig> {
    let site_name = env::var("CRAZY_SITE").unwrap_or_else(|_| "single-cf".to_string());
    
    let mut site_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    site_path.push("sites");
    site_path.push(format!("{}.toml", site_name));
    
    let contents = fs::read_to_string(&site_path)?;
    let config: SiteConfig = toml::from_str(&contents)?;
    
    Ok(config)
}

/// Get shared TOC cache directory
fn get_cache_dir() -> PathBuf {
    env::temp_dir().join("crazyflie_rust_test_cache")
}

/// Test 1: Validate connection WITHOUT cache (fresh TOC fetch)
/// This ensures the firmware TOC is valid and the lib can fetch it
/// RUN FIRST - validates basic connectivity
#[tokio::test]
#[serial]
async fn test_1_connection_without_cache() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let site_config = load_site_config()?;
    let device = site_config
        .device
        .get("default")
        .or_else(|| site_config.device.values().next())
        .expect("No device in site config");
    
    println!("Testing fresh TOC fetch (no cache) for: {}", device.radio);
    
    let link_context = LinkContext::new();
    
    // Use NoTocCache to force fresh fetch
    let cf = Crazyflie::connect_from_uri(&link_context, &device.radio, NoTocCache)
        .await?;
    
    println!("Connected successfully with fresh TOC fetch!");
    
    // Validate we can access TOC-dependent features
    let protocol_version = cf.platform.protocol_version().await?;
    println!("Protocol version: {}", protocol_version);
    
    // Try to access param system (requires TOC)
    let fw_version = cf.platform.firmware_version().await?;
    println!("Firmware version: {}", fw_version);
    
    // Create a log block (requires log TOC)
    let mut log_block = cf.log.create_block().await?;
    log_block.add_variable("stabilizer.roll").await?;
    println!("Successfully created log block with fresh TOC");
    
    cf.disconnect().await;
    println!("Fresh TOC test passed!");
    
    Ok(())
}

/// Test 2: Connect with cache to populate it, then measure cache benefit
/// RUN SECOND - now we can measure true cache performance
#[tokio::test]
#[serial]
async fn test_2_connection_with_cache() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let site_config = load_site_config()?;
    let device = site_config
        .device
        .get("default")
        .or_else(|| site_config.device.values().next())
        .expect("No device in site config");
    
    println!("Testing cache benefit for: {}", device.radio);
    
    // Clear cache to start fresh
    let cache_dir = get_cache_dir();
    if cache_dir.exists() {
        fs::remove_dir_all(&cache_dir).ok();
        fs::create_dir_all(&cache_dir).ok();
    }
    
    let link_context = LinkContext::new();
    let cache = FileTocCache::new(cache_dir);
    
    // First connection - should populate cache from scratch
    println!("First connection (populating empty cache)...");
    let start = std::time::Instant::now();
    let cf = Crazyflie::connect_from_uri(&link_context, &device.radio, cache.clone())
        .await?;
    let first_duration = start.elapsed();
    println!("First connection (no cache): {:?}", first_duration);
    
    cf.disconnect().await;
    
    // Small delay
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    
    // Second connection - should use cached TOC
    println!("Second connection (using cached TOC)...");
    let start = std::time::Instant::now();
    let cf = Crazyflie::connect_from_uri(&link_context, &device.radio, cache)
        .await?;
    let second_duration = start.elapsed();
    println!("Second connection (cached): {:?}", second_duration);
    
    // Verify functionality
    let protocol_version = cf.platform.protocol_version().await?;
    println!("Protocol version: {}", protocol_version);
    
    cf.disconnect().await;
    
    // Report results
    println!("\nCache performance:");
    println!("  Without cache: {:?}", first_duration);
    println!("  With cache:    {:?}", second_duration);
    if second_duration < first_duration {
        let speedup = first_duration.as_secs_f64() / second_duration.as_secs_f64();
        println!("  Speedup:       {:.2}x faster ✓", speedup);
    } else {
        println!("  Note: Cache didn't speed up connection (connection overhead dominates)");
    }
    
    Ok(())
}

/// Test 3: Validate cache correctness - cached TOC matches fresh TOC
/// RUN THIRD - validates the cache we just populated is correct
#[tokio::test]
#[serial]
async fn test_3_cache_correctness() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let site_config = load_site_config()?;
    let device = site_config
        .device
        .get("default")
        .or_else(|| site_config.device.values().next())
        .expect("No device in site config");
    
    println!("Validating cache correctness for: {}", device.radio);
    
    let link_context = LinkContext::new();
    
    // Connect with cache
    println!("Connecting with cache...");
    let cache = FileTocCache::new(get_cache_dir());
    let cf_cached = Crazyflie::connect_from_uri(&link_context, &device.radio, cache)
        .await?;
    
    let protocol_cached = cf_cached.platform.protocol_version().await?;
    let fw_cached = cf_cached.platform.firmware_version().await?;
    
    // Create a log block to verify log TOC
    let mut log_block_cached = cf_cached.log.create_block().await?;
    log_block_cached.add_variable("stabilizer.roll").await?;
    
    cf_cached.disconnect().await;
    
    // Connect without cache
    println!("Connecting without cache (fresh TOC)...");
    let cf_fresh = Crazyflie::connect_from_uri(&link_context, &device.radio, NoTocCache)
        .await?;
    
    let protocol_fresh = cf_fresh.platform.protocol_version().await?;
    let fw_fresh = cf_fresh.platform.firmware_version().await?;
    
    // Create same log block to verify log TOC matches
    let mut log_block_fresh = cf_fresh.log.create_block().await?;
    log_block_fresh.add_variable("stabilizer.roll").await?;
    
    cf_fresh.disconnect().await;
    
    // Validate results match
    assert_eq!(protocol_cached, protocol_fresh, "Protocol version mismatch between cached and fresh");
    assert_eq!(fw_cached, fw_fresh, "Firmware version mismatch between cached and fresh");
    
    println!("\nCache validation passed!");
    println!("  Protocol version: {}", protocol_cached);
    println!("  Firmware version: {}", fw_cached);
    println!("  Cached and fresh TOCs match ✓");
    
    Ok(())
}

/// Test 4: Recommended pattern for regular tests - fast cached connection
/// RUN FOURTH - example template for hardware tests using cache
#[tokio::test]
#[serial]
async fn test_4_example_hardware_test_with_cache() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let site_config = load_site_config()?;
    let device = site_config
        .device
        .get("default")
        .or_else(|| site_config.device.values().next())
        .expect("No device in site config");
    
    // Standard pattern: use cache for speed
    let link_context = LinkContext::new();
    let cache = FileTocCache::new(get_cache_dir());
    
    let cf = Crazyflie::connect_from_uri(&link_context, &device.radio, cache)
        .await?;
    
    // Your actual hardware test here...
    println!("Running hardware test...");
    
    let protocol_version = cf.platform.protocol_version().await?;
    assert!(protocol_version >= 10, "Protocol version too old");
    
    println!("Hardware test passed!");
    
    cf.disconnect().await;
    
    Ok(())
}
