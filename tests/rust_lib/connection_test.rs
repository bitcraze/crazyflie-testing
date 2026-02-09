use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_link::LinkContext;
use serde::Deserialize;
use serial_test::serial;
use std::env;
use std::fs;
use std::path::PathBuf;

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
    let site_name = env::var("CRAZY_SITE").unwrap_or_else(|_| "lh".to_string());
    
    let mut site_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    site_path.push("sites");
    site_path.push(format!("{}.toml", site_name));
    
    let contents = fs::read_to_string(&site_path)?;
    let config: SiteConfig = toml::from_str(&contents)?;
    
    Ok(config)
}

#[tokio::test]
#[serial]
async fn test_can_connect_to_crazyflie() -> Result<()> {
    // Initialize logging
    let _ = env_logger::builder().is_test(true).try_init();
    
    // Load site config (shared with Python tests)
    let site_config = load_site_config()?;
    
    // Get the first device
    let device = site_config
        .device
        .get("default")
        .expect("No 'default' device in site config");
    
    println!("Connecting to: {}", device.radio);
    
    // Create link context and connect (TOC cache is handled internally)
    let link_context = LinkContext::new();
    
    // Connect to Crazyflie (using NoTocCache for simplicity in tests)
    let cf = Crazyflie::connect_from_uri(&link_context, &device.radio, NoTocCache)
        .await?;
    
    println!("Connected successfully!");
    
    // Verify we can access platform info
    assert!(cf.platform.protocol_version().await? > 0);
    
    // Disconnect
    cf.disconnect().await;
    println!("Disconnected successfully!");
    
    Ok(())
}

#[tokio::test]
#[serial]
async fn test_connection_to_invalid_uri_fails() -> Result<()> {
    let link_context = LinkContext::new();
    
    // Try to connect to non-existent URI
    let result = Crazyflie::connect_from_uri(
        &link_context,
        "radio://0/60/2M/F00D2BEFED",
        NoTocCache
    ).await;
    
    // Should fail
    assert!(result.is_err());
    
    Ok(())
}
