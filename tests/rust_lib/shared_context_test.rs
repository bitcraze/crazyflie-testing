mod common;

use anyhow::Result;
use crazyflie_lib::{Crazyflie, NoTocCache, subsystems::log::LogPeriod};
use crazyflie_link::LinkContext;
use std::sync::Arc;

/// Test that a shared Crazyflie connection can handle multiple concurrent operations
#[tokio::test]
async fn test_concurrent_operations_on_shared_connection() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    for (name, device) in &devices {
        println!("\n[{}] Connecting to: {}", name, device.radio);

        let link_context = LinkContext::new();
        let cf = Arc::new(
            Crazyflie::connect_from_uri(&link_context, &device.radio, NoTocCache)
                .await?
        );

        println!("[{}] Connected! Now running concurrent operations...", name);

        let dev_name = name.clone();
        let cf1 = cf.clone();
        let task1 = tokio::spawn(async move {
            println!("[{}][Task 1] Reading protocol version...", dev_name);
            let version = cf1.platform.protocol_version().await?;
            println!("[{}][Task 1] Protocol version: {}", dev_name, version);
            Ok::<_, anyhow::Error>(version)
        });

        let dev_name = name.clone();
        let cf2 = cf.clone();
        let task2 = tokio::spawn(async move {
            println!("[{}][Task 2] Reading firmware version...", dev_name);
            let fw_version = cf2.platform.firmware_version().await?;
            println!("[{}][Task 2] Firmware version: {}", dev_name, fw_version);
            Ok::<_, anyhow::Error>(fw_version)
        });

        let dev_name = name.clone();
        let cf3 = cf.clone();
        let task3 = tokio::spawn(async move {
            println!("[{}][Task 3] Reading device type...", dev_name);
            let device_type = cf3.platform.device_type_name().await?;
            println!("[{}][Task 3] Device type: {}", dev_name, device_type);
            Ok::<_, anyhow::Error>(device_type)
        });

        let dev_name = name.clone();
        let cf4 = cf.clone();
        let task4 = tokio::spawn(async move {
            println!("[{}][Task 4] Creating and starting log block...", dev_name);

            let mut log_block = cf4.log.create_block().await?;
            log_block.add_variable("stabilizer.roll").await?;
            log_block.add_variable("stabilizer.pitch").await?;
            log_block.add_variable("stabilizer.yaw").await?;

            let period = LogPeriod::from_millis(100)?;
            let stream = log_block.start(period).await?;

            println!("[{}][Task 4] Log block started, reading one sample...", dev_name);

            let sample = stream.next().await?;
            println!("[{}][Task 4] Got log sample: {:?}", dev_name, sample);

            let _block = stream.stop().await?;
            println!("[{}][Task 4] Log block stopped", dev_name);

            Ok::<_, anyhow::Error>(())
        });

        let (r1, r2, r3, r4) = tokio::try_join!(task1, task2, task3, task4)?;

        let protocol_version = r1?;
        let firmware_version = r2?;
        let device_type = r3?;
        r4?;

        println!("\n[{}] All concurrent operations completed!", name);
        println!("[{}]   Protocol version: {}", name, protocol_version);
        println!("[{}]   Firmware version: {}", name, firmware_version);
        println!("[{}]   Device type: {}", name, device_type);

        assert!(protocol_version > 0);

        cf.disconnect().await;
        println!("[{}] Disconnected successfully!", name);
    }

    Ok(())
}
