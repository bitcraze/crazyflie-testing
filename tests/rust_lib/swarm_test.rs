use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache, TocCache};
use crazyflie_link::LinkContext;
use serde::Deserialize;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;
use std::collections::HashMap;
use std::sync::RwLock;
use std::time::Instant;

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
        if let Ok(lock) = self.memory_cache.read() {
            if let Some(toc) = lock.get(&crc32) {
                return Some(toc.clone());
            }
        }

        let path = self.cache_file_path(crc32);
        if let Ok(toc) = fs::read_to_string(&path) {
            if let Ok(mut lock) = self.memory_cache.write() {
                lock.insert(crc32, toc.clone());
            }
            return Some(toc);
        }

        None
    }

    fn store_toc(&self, crc32: u32, toc: &str) {
        if let Ok(mut lock) = self.memory_cache.write() {
            lock.insert(crc32, toc.to_string());
        }

        let path = self.cache_file_path(crc32);
        fs::write(path, toc).ok();
    }
}

/// Get shared TOC cache directory
fn get_cache_dir() -> PathBuf {
    env::temp_dir().join("crazyflie_rust_test_cache")
}

/// Check if TOC caching is enabled via USE_TOC_CACHE env var
fn use_toc_cache() -> bool {
    env::var("USE_TOC_CACHE").map(|v| v == "1" || v.to_lowercase() == "true").unwrap_or(false)
}

/// Connect to a Crazyflie, optionally using TOC cache
async fn connect_crazyflie(
    ctx: &LinkContext,
    uri: &str,
) -> Result<Crazyflie> {
    if use_toc_cache() {
        let cache = FileTocCache::new(get_cache_dir());
        Ok(Crazyflie::connect_from_uri(ctx, uri, cache).await?)
    } else {
        Ok(Crazyflie::connect_from_uri(ctx, uri, NoTocCache).await?)
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
    let site_name = env::var("CRAZY_SITE").unwrap_or_else(|_| "desk-swarm".to_string());
    
    let mut site_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    site_path.push("sites");
    site_path.push(format!("{}.toml", site_name));
    
    let contents = fs::read_to_string(&site_path)?;
    let config: SiteConfig = toml::from_str(&contents)?;
    
    Ok(config)
}

/// Test Pattern 1: Connect to all drones in parallel with shared LinkContext
#[tokio::test]
async fn test_swarm_parallel_connections() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    let start = Instant::now();

    let site_config = load_site_config()?;

    // Skip if only one device
    if site_config.device.len() <= 1 {
        println!("Skipping swarm test - only {} device(s) configured", site_config.device.len());
        return Ok(());
    }

    println!("Connecting to {} drones in parallel...", site_config.device.len());
    
    // Shared LinkContext for all drones
    let link_context = Arc::new(LinkContext::new());
    
    // Connect to all drones concurrently
    let mut connection_tasks = vec![];
    
    let cache = if use_toc_cache() {
        println!("Using TOC cache");
        Some(FileTocCache::new(get_cache_dir()))
    } else {
        None
    };

    for (name, device) in &site_config.device {
        let name = name.clone();
        let uri = device.radio.clone();
        let ctx = link_context.clone();
        let cache = cache.clone();

        let task = tokio::spawn(async move {
            println!("[{}] Connecting to {}...", name, uri);
            let cf = if let Some(cache) = cache {
                Crazyflie::connect_from_uri(&ctx, &uri, cache).await?
            } else {
                Crazyflie::connect_from_uri(&ctx, &uri, NoTocCache).await?
            };
            println!("[{}] Connected!", name);
            Ok::<_, anyhow::Error>((name, cf))
        });

        connection_tasks.push(task);
    }
    
    // Wait for all connections
    let mut crazyflies = vec![];
    for task in connection_tasks {
        crazyflies.push(task.await??);
    }
    
    println!("\nAll {} drones connected! Running concurrent tests...", crazyflies.len());
    
    // Run a test on all drones concurrently
    let mut test_tasks = vec![];
    
    for (name, cf) in crazyflies {
        let cf = Arc::new(cf);
        let task = tokio::spawn(async move {
            let version = cf.platform.protocol_version().await?;
            let fw = cf.platform.firmware_version().await?;
            println!("[{}] Protocol v{}, Firmware: {}", name, version, fw);
            
            cf.disconnect().await;
            println!("[{}] Disconnected", name);
            
            Ok::<_, anyhow::Error>(())
        });
        
        test_tasks.push(task);
    }
    
    // Wait for all tests
    for task in test_tasks {
        task.await??;
    }

    println!("\nSwarm test completed successfully in {:?}", start.elapsed());

    Ok(())
}

/// Test Pattern 2: Test each drone sequentially (easier debugging)
#[tokio::test]
async fn test_each_drone_sequentially() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    let start = Instant::now();

    let site_config = load_site_config()?;
    let link_context = LinkContext::new();

    println!("Testing {} drone(s) sequentially...", site_config.device.len());
    if use_toc_cache() {
        println!("Using TOC cache");
    }

    for (name, device) in &site_config.device {
        println!("\n--- Testing drone: {} ---", name);
        println!("URI: {}", device.radio);

        // Connect
        let cf = connect_crazyflie(&link_context, &device.radio).await?;
        println!("[{}] Connected", name);
        
        // Run tests
        let protocol_version = cf.platform.protocol_version().await?;
        let firmware_version = cf.platform.firmware_version().await?;
        let device_type = cf.platform.device_type_name().await?;
        
        println!("[{}] Protocol version: {}", name, protocol_version);
        println!("[{}] Firmware version: {}", name, firmware_version);
        println!("[{}] Device type: {}", name, device_type);
        
        assert!(protocol_version > 0, "Invalid protocol version");
        
        // Disconnect
        cf.disconnect().await;
        println!("[{}] Disconnected", name);
    }

    println!("\nSequential test completed successfully in {:?}", start.elapsed());

    Ok(())
}

/// Test Pattern 3: Connect all, then test each one by one
#[tokio::test]
async fn test_swarm_connect_all_then_test_individually() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    let start = Instant::now();

    let site_config = load_site_config()?;

    // Skip if only one device
    if site_config.device.len() <= 1 {
        println!("Skipping swarm test - only {} device(s) configured", site_config.device.len());
        return Ok(());
    }

    println!("Connecting to all {} drones...", site_config.device.len());

    let link_context = Arc::new(LinkContext::new());

    let cache = if use_toc_cache() {
        println!("Using TOC cache");
        Some(FileTocCache::new(get_cache_dir()))
    } else {
        None
    };

    // Connect to all drones in parallel
    let mut connection_tasks = vec![];

    for (name, device) in &site_config.device {
        let name = name.clone();
        let uri = device.radio.clone();
        let ctx = link_context.clone();
        let cache = cache.clone();

        let task = tokio::spawn(async move {
            let cf = if let Some(cache) = cache {
                Crazyflie::connect_from_uri(&ctx, &uri, cache).await?
            } else {
                Crazyflie::connect_from_uri(&ctx, &uri, NoTocCache).await?
            };
            println!("[{}] Connected", name);
            Ok::<_, anyhow::Error>((name, Arc::new(cf)))
        });

        connection_tasks.push(task);
    }
    
    let mut crazyflies = HashMap::new();
    for task in connection_tasks {
        let (name, cf) = task.await??;
        crazyflies.insert(name, cf);
    }
    
    println!("\nAll drones connected! Testing each one sequentially...");
    
    // Now test each drone sequentially (easier to isolate failures)
    for (name, cf) in &crazyflies {
        println!("\n--- Testing {} ---", name);
        
        let protocol_version = cf.platform.protocol_version().await?;
        let firmware_version = cf.platform.firmware_version().await?;
        
        println!("[{}] Protocol v{}, Firmware: {}", name, protocol_version, firmware_version);
        
        assert!(protocol_version > 0);
    }
    
    // Disconnect all
    println!("\nDisconnecting all drones...");
    for (name, cf) in crazyflies {
        cf.disconnect().await;
        println!("[{}] Disconnected", name);
    }

    println!("\nSwarm test completed in {:?}", start.elapsed());

    Ok(())
}
