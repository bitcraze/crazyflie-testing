mod common;

use anyhow::Result;
use crazyflie_lib::subsystems::log::LogPeriod;
use crazyflie_lib::{Crazyflie, Value};
use crazyflie_link::LinkContext;
use serial_test::serial;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

/// Variable names for a max-payload log block (26 bytes).
/// 6 x f32 (24 bytes) + 2 x u8 (2 bytes) = 26 bytes total.
const MAX_BYTES_VARIABLES: &[&str] = &[
    "stabilizer.roll",   // f32: 4 bytes
    "stabilizer.pitch",  // f32: 4 bytes
    "stabilizer.yaw",    // f32: 4 bytes
    "stabilizer.thrust", // f32: 4 bytes
    "gyro.xVariance",    // f32: 4 bytes
    "gyro.yVariance",    // f32: 4 bytes
    "radio.rssi",        // u8:  1 byte
    "pm.state",          // u8:  1 byte
];

/// Create a log block at maximum payload (26 bytes).
async fn create_log_block_max_bytes(cf: &Crazyflie) -> Result<crazyflie_lib::subsystems::log::LogBlock> {
    let mut block = cf.log.create_block().await?;
    for var in MAX_BYTES_VARIABLES {
        block.add_variable(var).await?;
    }
    Ok(block)
}

/// Assert that a log data sample contains all expected variables from the max-bytes block.
fn assert_variables_present(data: &HashMap<String, Value>) {
    assert_eq!(
        data.len(),
        MAX_BYTES_VARIABLES.len(),
        "Expected {} variables, got {}",
        MAX_BYTES_VARIABLES.len(),
        data.len()
    );
    for var in MAX_BYTES_VARIABLES {
        assert!(
            data.contains_key(*var),
            "Missing variable '{}' in log data. Present: {:?}",
            var,
            data.keys().collect::<Vec<_>>()
        );
    }
}

/// Assert that `actual` is within `max_diff_percent`% of `expected`.
fn assert_within_percentage(expected: f64, actual: f64, max_diff_percent: f64) {
    let max_diff = expected * (max_diff_percent / 100.0);
    assert!(
        actual >= expected - max_diff,
        "Rate too low: expected >= {:.1} ({}% below {:.1}), got {:.1}",
        expected - max_diff,
        max_diff_percent,
        expected,
        actual
    );
    assert!(
        actual <= expected + max_diff,
        "Rate too high: expected <= {:.1} ({}% above {:.1}), got {:.1}",
        expected + max_diff,
        max_diff_percent,
        expected,
        actual
    );
}

/// Log at 100Hz for 5 seconds, verify we receive data at the expected rate
/// and that all variables are present in each sample.
#[tokio::test]
#[serial]
async fn test_log_async() -> Result<()> {
    common::init_logging();
    let reqs = common::load_logging_requirements()?;
    let expected_rate = reqs.basic.max_rate as f64;
    let period_ms = (1000.0 / expected_rate) as u64;
    let duration_secs = 5.0_f64;

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    for (name, device) in &devices {
        println!("\n[{}] Testing log async @ {}", name, device.radio);
        let link_context = LinkContext::new();
        let cf = common::connect_crazyflie(&link_context, &device.radio).await?;

        let block = create_log_block_max_bytes(&cf).await?;
        let period = LogPeriod::from_millis(period_ms)?;
        let stream = block.start(period).await?;

        let mut rows = 0u64;
        let start = Instant::now();

        while start.elapsed().as_secs_f64() < duration_secs {
            let sample = stream.next().await?;
            rows += 1;
            assert_variables_present(&sample.data);
        }

        let _block = stream.stop().await?;

        let actual_rate = rows as f64 / duration_secs;
        println!(
            "[{}] expected={:.1} Hz, actual={:.1} Hz, rows={}",
            name, expected_rate, actual_rate, rows
        );
        assert_within_percentage(expected_rate, actual_rate, 3.0);

        cf.disconnect().await;
    }

    Ok(())
}

/// Read exactly max_rate (100) samples at 10ms period, verifying all
/// variables are present in each sample.
#[tokio::test]
#[serial]
async fn test_log_sync() -> Result<()> {
    common::init_logging();
    let reqs = common::load_logging_requirements()?;
    let max_rate = reqs.basic.max_rate;

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    for (name, device) in &devices {
        println!("\n[{}] Testing log sync @ {}", name, device.radio);
        let link_context = LinkContext::new();
        let cf = common::connect_crazyflie(&link_context, &device.radio).await?;

        let block = create_log_block_max_bytes(&cf).await?;
        let period = LogPeriod::from_millis(10)?;
        let stream = block.start(period).await?;

        for i in 0..max_rate {
            let sample = stream.next().await?;
            assert_variables_present(&sample.data);
            if i % 25 == 0 {
                println!("[{}] read sample {}/{}", name, i + 1, max_rate);
            }
        }

        let _block = stream.stop().await?;
        println!("[{}] successfully read {} samples", name, max_rate);

        cf.disconnect().await;
    }

    Ok(())
}

/// Verify that adding more bytes than max_payload (26) to a single block fails.
#[tokio::test]
#[serial]
async fn test_log_too_much_per_block() -> Result<()> {
    common::init_logging();

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    for (name, device) in &devices {
        println!("\n[{}] Testing log too much per block @ {}", name, device.radio);
        let link_context = LinkContext::new();
        let cf = common::connect_crazyflie(&link_context, &device.radio).await?;

        // Create a block at exactly 26 bytes (max payload)
        let mut block = create_log_block_max_bytes(&cf).await?;

        // Adding one more byte (radio.rssi = u8 = 1 byte) should fail at 27 bytes
        let result = block.add_variable("radio.rssi").await;
        assert!(
            result.is_err(),
            "[{}] Adding variable beyond max_payload (26 bytes) should fail",
            name
        );
        println!("[{}] got expected error: {}", name, result.unwrap_err());

        cf.disconnect().await;
    }

    Ok(())
}

/// Verify that exceeding the maximum number of log variables (128 total
/// across all active blocks) results in an error.
#[tokio::test]
#[serial]
async fn test_log_too_many_variables() -> Result<()> {
    common::init_logging();
    let reqs = common::load_logging_requirements()?;
    let max_blocks = reqs.blocks.max as usize;

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    // 9 small variables per block. 16 blocks * 9 vars = 144 > 128 limit.
    let small_vars = [
        "sys.canfly",
        "sys.isFlying",
        "sys.isTumbled",
        "radio.rssi",
        "pm.state",
        "pm.batteryLevel",
        "pm.vbat",
        "pm.chg",
        "sys.armed",
    ];

    for (name, device) in &devices {
        println!("\n[{}] Testing log too many variables @ {}", name, device.radio);
        let link_context = LinkContext::new();
        let cf = common::connect_crazyflie(&link_context, &device.radio).await?;

        let mut streams = Vec::new();
        let mut hit_error = false;

        for i in 0..max_blocks {
            let block_result = async {
                let mut block = cf.log.create_block().await?;
                for var in &small_vars {
                    block.add_variable(var).await?;
                }
                let period = LogPeriod::from_millis(100)?;
                let stream = block.start(period).await?;
                Ok::<_, anyhow::Error>(stream)
            }
            .await;

            match block_result {
                Ok(stream) => {
                    streams.push(stream);
                    println!(
                        "[{}] Block {} started ({} total vars)",
                        name,
                        i,
                        (i + 1) * small_vars.len()
                    );
                }
                Err(e) => {
                    println!("[{}] Expected error at block {}: {}", name, i, e);
                    hit_error = true;
                    break;
                }
            }
        }

        // Cleanup: stop all streams that were started
        for stream in streams {
            let _ = stream.stop().await;
        }

        assert!(
            hit_error,
            "[{}] Should have failed when exceeding 128 variable limit",
            name
        );

        cf.disconnect().await;
    }

    Ok(())
}

/// Test exceeding the maximum number of active log blocks.
/// Currently causes deadlock in the Rust crazyflie-lib implementation.
#[tokio::test]
#[serial]
#[ignore = "causes deadlock in current Rust crazyflie-lib implementation"]
async fn test_log_too_many_blocks() -> Result<()> {
    // TODO: implement when the deadlock in crazyflie-lib is fixed
    Ok(())
}

/// Stress test: run multiple log blocks at high packet rate (300 pkt/s total).
/// Uses tokio::spawn per stream with atomic counters for idiomatic async Rust.
#[tokio::test]
#[serial]
async fn test_log_stress() -> Result<()> {
    common::init_logging();
    let reqs = common::load_logging_requirements()?;

    let period_ms = 10_u64; // 10ms = 100 Hz per block
    let expected_rate_per_block = 1000.0 / period_ms as f64;
    let expected_total_rate = reqs.rate.limit_low as f64;
    let nr_of_log_blocks = (expected_total_rate / expected_rate_per_block) as usize;
    let duration_secs = 10.0_f64;

    let site_config = common::load_site_config()?;
    let devices = common::get_devices(&site_config);
    assert!(!devices.is_empty(), "No devices in site config");

    for (name, device) in &devices {
        println!("\n[{}] Testing log stress @ {}", name, device.radio);
        let link_context = LinkContext::new();
        let cf = common::connect_crazyflie(&link_context, &device.radio).await?;

        // Create and start all blocks at max payload
        let mut streams = Vec::new();
        for i in 0..nr_of_log_blocks {
            let block = create_log_block_max_bytes(&cf).await?;
            let period = LogPeriod::from_millis(period_ms)?;
            let stream = block.start(period).await?;
            println!("[{}] Started stress block {}/{}", name, i + 1, nr_of_log_blocks);
            streams.push(stream);
        }

        // Atomic counters for each block
        let counters: Vec<Arc<AtomicU64>> = (0..nr_of_log_blocks)
            .map(|_| Arc::new(AtomicU64::new(0)))
            .collect();

        let start = Instant::now();
        let mut handles = Vec::new();

        // Spawn a counting task per stream
        for (i, stream) in streams.into_iter().enumerate() {
            let counter = counters[i].clone();
            let handle = tokio::spawn(async move {
                while let Ok(_sample) = stream.next().await {
                    counter.fetch_add(1, Ordering::Relaxed);
                }
            });
            handles.push(handle);
        }

        // Wait for test duration
        tokio::time::sleep(tokio::time::Duration::from_secs_f64(duration_secs)).await;
        let elapsed = start.elapsed().as_secs_f64();

        // Abort counting tasks; dropping LogStream triggers firmware cleanup
        for handle in &handles {
            handle.abort();
        }
        for handle in handles {
            let _ = handle.await;
        }

        // Assert per-block and total rates
        let mut total_packets = 0u64;
        for i in 0..nr_of_log_blocks {
            let count = counters[i].load(Ordering::Relaxed);
            total_packets += count;
            let actual_rate = count as f64 / elapsed;
            println!(
                "[{}] Block {}: {} packets in {:.1}s = {:.1} Hz (expected {:.1} Hz)",
                name, i, count, elapsed, actual_rate, expected_rate_per_block
            );
            assert_within_percentage(expected_rate_per_block, actual_rate, 3.0);
        }

        let actual_total_rate = total_packets as f64 / elapsed;
        println!(
            "[{}] Total: {} packets in {:.1}s = {:.1} Hz (expected {:.1} Hz)",
            name, total_packets, elapsed, actual_total_rate, expected_total_rate
        );
        assert_within_percentage(expected_total_rate, actual_total_rate, 3.0);

        cf.disconnect().await;
    }

    Ok(())
}
