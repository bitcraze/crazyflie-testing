mod common;

use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_link::LinkContext;
use serial_test::serial;

#[tokio::test]
#[serial]
async fn test_can_connect_to_crazyflie() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    for (name, device) in &devices {
        println!("\n[{}] Connecting to: {}", name, device.radio);

        let link_context = LinkContext::new();
        let cf = Crazyflie::connect_from_uri(&link_context, &device.radio, NoTocCache)
            .await?;

        println!("[{}] Connected successfully!", name);

        assert!(cf.platform.protocol_version().await? > 0);

        cf.disconnect().await;
        println!("[{}] Disconnected successfully!", name);
    }

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
