mod common;

use anyhow::Result;
use crazyflie_link::LinkContext;
use futures::StreamExt;
use std::sync::Arc;

/// Connect to all drones concurrently, set ring.effect to a known value,
/// read it back, and restore the original value.
///
/// This is the most fundamental param contract: write is confirmed by the
/// firmware and a subsequent read returns what was written.
#[tokio::test]
async fn test_param_get_set() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    let link_context = Arc::new(LinkContext::new());

    let tasks: Vec<_> = devices
        .into_iter()
        .map(|(name, device)| {
            let ctx = link_context.clone();
            let task_name = name.clone();
            let task = tokio::spawn(async move {
                let cf = common::connect_crazyflie(&ctx, &device.radio).await?;

                let param = "ring.effect";

                // Read the current value so we can restore it afterwards
                let original: u8 = cf.param.get(param).await?;

                // Write a different value
                let target: u8 = if original == 6 { 7 } else { 6 };
                cf.param.set(param, target).await?;

                // Read back and verify
                let readback: u8 = cf.param.get(param).await?;
                assert_eq!(
                    readback, target,
                    "[{}] param readback mismatch: wrote {} but got {}",
                    name, target, readback
                );

                // Restore original value
                cf.param.set(param, original).await?;

                cf.disconnect().await;
                println!("[{}] ok", name);
                Ok::<_, anyhow::Error>(())
            });
            (task_name, task)
        })
        .collect();

    for (name, task) in tasks {
        task.await?
            .map_err(|e| anyhow::anyhow!("[{}] {}", name, e))?;
    }

    Ok(())
}

/// Verify that watch_change() emits a notification when a parameter is set.
///
/// The watcher must be registered before the set, and the notification must
/// carry the correct parameter name and value. Since set() only returns after
/// the firmware confirms the write (which is also when the notification fires),
/// the event should already be in the stream by the time we poll it.
#[tokio::test]
async fn test_param_watch_change() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    let link_context = Arc::new(LinkContext::new());

    let tasks: Vec<_> = devices
        .into_iter()
        .map(|(name, device)| {
            let ctx = link_context.clone();
            let task_name = name.clone();
            let task = tokio::spawn(async move {
                let cf = common::connect_crazyflie(&ctx, &device.radio).await?;

                let param = "ring.effect";

                let original: u8 = cf.param.get(param).await?;
                let target: u8 = if original == 6 { 7 } else { 6 };

                // Register watcher before the set
                let mut stream = cf.param.watch_change().await;

                cf.param.set(param, target).await?;

                // The notification should already be buffered in the stream
                let (notif_name, notif_value) = stream
                    .next()
                    .await
                    .ok_or_else(|| anyhow::anyhow!("[{}] watch_change stream closed unexpectedly", name))?;

                assert_eq!(
                    notif_name, param,
                    "[{}] notification for wrong param: expected {}, got {}",
                    name, param, notif_name
                );
                let notif_u8: u8 = notif_value.try_into().map_err(|e| {
                    anyhow::anyhow!("[{}] notification value has wrong type: {:?}", name, e)
                })?;
                assert_eq!(
                    notif_u8, target,
                    "[{}] notification value mismatch: expected {}, got {}",
                    name, target, notif_u8
                );

                cf.param.set(param, original).await?;

                cf.disconnect().await;
                println!("[{}] ok", name);
                Ok::<_, anyhow::Error>(())
            });
            (task_name, task)
        })
        .collect();

    for (name, task) in tasks {
        task.await?
            .map_err(|e| anyhow::anyhow!("[{}] {}", name, e))?;
    }

    Ok(())
}

/// Verify that setting a read-only parameter returns an error.
///
/// pm.vbat is a battery voltage sensor reading — always read-only on all CF2 hardware.
#[tokio::test]
async fn test_param_read_only_enforced() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    let link_context = Arc::new(LinkContext::new());

    let tasks: Vec<_> = devices
        .into_iter()
        .map(|(name, device)| {
            let ctx = link_context.clone();
            let task_name = name.clone();
            let task = tokio::spawn(async move {
                let cf = common::connect_crazyflie(&ctx, &device.radio).await?;

                let result = cf.param.set("pm.vbat", 0.0f32).await;

                assert!(
                    result.is_err(),
                    "[{}] expected error when setting read-only param pm.vbat, but got Ok",
                    name
                );

                cf.disconnect().await;
                println!("[{}] ok", name);
                Ok::<_, anyhow::Error>(())
            });
            (task_name, task)
        })
        .collect();

    for (name, task) in tasks {
        task.await?
            .map_err(|e| anyhow::anyhow!("[{}] {}", name, e))?;
    }

    Ok(())
}
