use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache, subsystems::log::LogPeriod};
use crazyflie_link::LinkContext;
use serde::Deserialize;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;

/// Site configuration format matching sites/*.toml
#[derive(Debug, Deserialize)]
struct SiteConfig {
    version: u32,
    device: std::collections::HashMap<String, DeviceConfig>,
}

#[derive(Debug, Deserialize)]
struct DeviceConfig {
    radio: String,
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

/// Test that a shared Crazyflie connection can handle multiple concurrent operations
#[tokio::test]
async fn test_concurrent_operations_on_shared_connection() -> Result<()> {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let site_config = load_site_config()?;
    let device = site_config
        .device
        .get("default")
        .expect("No 'default' device in site config");
    
    println!("Connecting to: {}", device.radio);
    
    let link_context = LinkContext::new();
    let cf = Arc::new(
        Crazyflie::connect_from_uri(&link_context, &device.radio, NoTocCache)
            .await?
    );
    
    println!("Connected! Now running concurrent operations...");
    
    // Spawn multiple concurrent tasks that all use the same connection
    let cf1 = cf.clone();
    let task1 = tokio::spawn(async move {
        println!("[Task 1] Reading protocol version...");
        let version = cf1.platform.protocol_version().await?;
        println!("[Task 1] Protocol version: {}", version);
        Ok::<_, anyhow::Error>(version)
    });
    
    let cf2 = cf.clone();
    let task2 = tokio::spawn(async move {
        println!("[Task 2] Reading firmware version...");
        let fw_version = cf2.platform.firmware_version().await?;
        println!("[Task 2] Firmware version: {}", fw_version);
        Ok::<_, anyhow::Error>(fw_version)
    });
    
    let cf3 = cf.clone();
    let task3 = tokio::spawn(async move {
        println!("[Task 3] Reading device type...");
        let device_type = cf3.platform.device_type_name().await?;
        println!("[Task 3] Device type: {}", device_type);
        Ok::<_, anyhow::Error>(device_type)
    });
    
    let cf4 = cf.clone();
    let task4 = tokio::spawn(async move {
        println!("[Task 4] Creating and starting log block...");
        
        // Create a log block
        let mut log_block = cf4.log.create_block().await?;
        log_block.add_variable("stabilizer.roll").await?;
        log_block.add_variable("stabilizer.pitch").await?;
        log_block.add_variable("stabilizer.yaw").await?;
        
        // Start logging
        let period = LogPeriod::from_millis(100)?;
        let stream = log_block.start(period).await?;
        
        println!("[Task 4] Log block started, reading one sample...");
        
        // Read one sample
        let sample = stream.next().await?;
        println!("[Task 4] Got log sample: {:?}", sample);
        
        // Stop logging
        let _block = stream.stop().await?;
        println!("[Task 4] Log block stopped");
        
        Ok::<_, anyhow::Error>(())
    });
    
    // Wait for all tasks to complete
    let (r1, r2, r3, r4) = tokio::try_join!(task1, task2, task3, task4)?;
    
    // Check results
    let protocol_version = r1?;
    let firmware_version = r2?;
    let device_type = r3?;
    r4?;
    
    println!("\nAll concurrent operations completed successfully!");
    println!("  Protocol version: {}", protocol_version);
    println!("  Firmware version: {}", firmware_version);
    println!("  Device type: {}", device_type);
    
    assert!(protocol_version > 0);
    
    // Disconnect
    cf.disconnect().await;
    println!("Disconnected successfully!");
    
    Ok(())
}
