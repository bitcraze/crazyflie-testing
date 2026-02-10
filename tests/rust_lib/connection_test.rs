mod common;

use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_link::LinkContext;
use serial_test::serial;

#[tokio::test]
#[serial]
async fn test_can_connect_to_crazyflie() -> Result<()> {
    common::init_logging();

    // Load site config (shared with Python tests)
    let site_config = common::load_site_config()?;

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
