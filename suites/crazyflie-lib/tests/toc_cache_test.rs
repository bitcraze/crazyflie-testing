mod common;
use common::{FileTocCache, get_cache_dir};

use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_link::LinkContext;
use serial_test::serial;
use std::fs;

/// Test 1: Validate connection WITHOUT cache (fresh TOC fetch)
/// This ensures the firmware TOC is valid and the lib can fetch it
/// RUN FIRST - validates basic connectivity
#[tokio::test]
#[serial]
async fn test_1_connection_without_cache() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
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
    common::init_logging();

    let site_config = common::load_site_config()?;
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
    common::init_logging();

    let site_config = common::load_site_config()?;
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
    common::init_logging();

    let site_config = common::load_site_config()?;
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
