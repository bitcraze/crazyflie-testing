mod common;
use common::{FileTocCache, get_cache_dir, use_toc_cache, connect_crazyflie};

use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

/// Test Pattern 1: Connect to all drones in parallel with shared LinkContext
#[tokio::test]
async fn test_swarm_parallel_connections() -> Result<()> {
    common::init_logging();
    let start = Instant::now();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);

    // Skip if only one device
    if devices.len() <= 1 {
        println!("Skipping swarm test - only {} device(s) configured", devices.len());
        return Ok(());
    }

    println!("Connecting to {} drones in parallel...", devices.len());

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

    for (name, device) in &devices {
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
    common::init_logging();
    let start = Instant::now();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    let link_context = LinkContext::new();

    println!("Testing {} drone(s) sequentially...", devices.len());
    if use_toc_cache() {
        println!("Using TOC cache");
    }

    for (name, device) in &devices {
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
    common::init_logging();
    let start = Instant::now();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);

    // Skip if only one device
    if devices.len() <= 1 {
        println!("Skipping swarm test - only {} device(s) configured", devices.len());
        return Ok(());
    }

    println!("Connecting to all {} drones...", devices.len());

    let link_context = Arc::new(LinkContext::new());

    let cache = if use_toc_cache() {
        println!("Using TOC cache");
        Some(FileTocCache::new(get_cache_dir()))
    } else {
        None
    };

    // Connect to all drones in parallel
    let mut connection_tasks = vec![];

    for (name, device) in &devices {
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
