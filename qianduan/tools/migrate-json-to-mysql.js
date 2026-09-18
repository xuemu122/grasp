#!/usr/bin/env node
/**
 * Export the current JSON catalog and operations stores as MySQL SQL.
 * This is a staging import only: server.js continues to read JSON until the
 * database has been reviewed and an explicit read-path migration is made.
 *
 * Usage:
 *   node tools/migrate-json-to-mysql.js > /tmp/grasp_catalog.sql
 *   mysql ... < /tmp/grasp_catalog.sql
 */
const fs = require('node:fs');
const path = require('node:path');

const base = path.join(__dirname, '..');
const productsFile = process.env.PRODUCTS_DB_FILE || path.join(base, 'data', 'products.json');
const operationsFile = process.env.OPERATIONS_DB_FILE || path.join(base, 'data', 'operations.json');
const database = String(process.env.MYSQL_DATABASE || 'grasp_catalog').trim();
if (!/^[A-Za-z0-9_]+$/.test(database)) throw new Error('MYSQL_DATABASE 只能包含字母、数字和下划线');

function readJson(file, fallback) {
    if (!fs.existsSync(file)) return fallback;
    const value = JSON.parse(fs.readFileSync(file, 'utf8'));
    return value && typeof value === 'object' ? value : fallback;
}

function sqlString(value) {
    if (value === null) return 'NULL';
    if (value === undefined) return "''";
    return `'${String(value).replace(/\\/g, '\\\\').replace(/'/g, "''").replace(/\0/g, '')}'`;
}

function sqlNumber(value, fallback = 0) {
    const number = Number(value);
    return Number.isFinite(number) ? String(number) : String(fallback);
}

function sqlDate(value) {
    if (!value) return 'NULL';
    const date = new Date(value);
    return Number.isNaN(date.getTime())
        ? 'NULL'
        : sqlString(date.toISOString().slice(0, 19).replace('T', ' '));
}

function emit(value = '') { process.stdout.write(`${value}\n`); }

const productsStore = readJson(productsFile, { products: [] });
const ordersFile = process.env.ORDERS_DB_FILE || path.join(path.dirname(productsFile), 'orders.json');
const ordersStore = readJson(ordersFile, { orders: [] });
const operations = readJson(operationsFile, { stores: [], devices: [], merchants: [] });
const products = Array.isArray(productsStore.products) ? productsStore.products : [];
const orders = Array.isArray(ordersStore.orders) ? ordersStore.orders : [];
const stores = Array.isArray(operations.stores) ? operations.stores : [];
const devices = Array.isArray(operations.devices) ? operations.devices : [];
const merchants = Array.isArray(operations.merchants) ? operations.merchants : [];
const merchantStores = merchants.flatMap((merchant) => (Array.isArray(merchant.storeIds) ? merchant.storeIds : [])
    .map((storeId) => ({ merchantId: merchant.id, storeId })));
const orderItems = orders.flatMap((order) => (Array.isArray(order.items) ? order.items : [])
    .map((item, index) => ({ order, item, lineNo: index + 1 })));
const orderRefunds = orders.flatMap((order) => {
    const history = Array.isArray(order.refundHistory) && order.refundHistory.length > 0
        ? order.refundHistory
        : (order.refundId || order.refundStatus ? [{
            refundId: order.refundId,
            refundRequestId: order.refundRequestId,
            refundAmount: order.refundAmount,
            refundStatus: order.refundStatus,
            reason: order.refundReason,
            trigger: order.refundTrigger,
            refundItems: order.refundItems,
            requestedAt: order.refundRequestedAt,
            lastAttemptAt: order.refundLastAttemptAt,
            attemptCount: order.refundAttemptCount,
            completedAt: order.refundedAt || order.partiallyRefundedAt
        }] : []);
    return history.map((refund, index) => ({ order, refund, sequence: index + 1 }));
});
const inventoryFile = process.env.INVENTORY_SCAN_FILE || path.join(path.dirname(productsFile), 'inventory_scan.json');
const chassisSettingsFile = process.env.CHASSIS_SETTINGS_FILE || path.join(path.dirname(productsFile), 'chassis_settings.json');
const outboxFile = process.env.ORDER_CALLBACK_OUTBOX_FILE || path.join(path.dirname(productsFile), 'order_callback_outbox.json');
const inventory = readJson(inventoryFile, { shelves: [], last_scan: null });
const chassisSettings = readJson(chassisSettingsFile, null);
const outbox = readJson(outboxFile, { events: [] });
const inventoryShelves = Array.isArray(inventory.shelves) ? inventory.shelves : [];
const inventoryTask = inventory.last_scan && typeof inventory.last_scan === 'object' ? inventory.last_scan : null;
const outboxEvents = Array.isArray(outbox.events) ? outbox.events : [];
const reservedOrders = productsStore.reservedOrders && typeof productsStore.reservedOrders === 'object' ? productsStore.reservedOrders : {};
const completedOrders = productsStore.completedOrders && typeof productsStore.completedOrders === 'object' ? productsStore.completedOrders : {};
const deviceEvents = devices.flatMap((device) => (Array.isArray(device.events) ? device.events : [])
    .map((event, index) => ({ deviceId: device.id, event, sequence: index + 1 })));
const migratedAt = sqlString(new Date().toISOString().slice(0, 19).replace('T', ' '));

emit('-- Generated from JSON stores. The Node service still reads JSON.');
emit('SET NAMES utf8mb4;');
emit(`CREATE DATABASE IF NOT EXISTS \`${database}\` CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;`);
emit(`USE \`${database}\`;`);
emit(`CREATE TABLE IF NOT EXISTS stores (
  id VARCHAR(64) NOT NULL,
  name VARCHAR(128) NOT NULL,
  code VARCHAR(64) NOT NULL,
  address VARCHAR(255) NOT NULL,
  contact VARCHAR(128) NOT NULL,
  phone VARCHAR(64) NOT NULL,
  status VARCHAR(32) NOT NULL,
  notes TEXT NOT NULL,
  created_at DATETIME NULL,
  updated_at DATETIME NULL,
  PRIMARY KEY (id),
  UNIQUE KEY uk_stores_code (code)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS devices (
  id VARCHAR(64) NOT NULL,
  sn VARCHAR(128) NULL,
  name VARCHAR(128) NOT NULL,
  model VARCHAR(128) NOT NULL,
  status VARCHAR(32) NOT NULL,
  store_id VARCHAR(64) NULL,
  store_name VARCHAR(128) NULL,
  location VARCHAR(255) NOT NULL,
  notes TEXT NOT NULL,
  last_seen_at DATETIME NULL,
  created_at DATETIME NULL,
  updated_at DATETIME NULL,
  PRIMARY KEY (id),
  UNIQUE KEY uk_devices_sn (sn),
  KEY idx_devices_store (store_id),
  CONSTRAINT fk_devices_store FOREIGN KEY (store_id) REFERENCES stores(id) ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS products (
  id INT NOT NULL,
  name VARCHAR(255) NOT NULL,
  spec VARCHAR(255) NOT NULL,
  packaging VARCHAR(128) NOT NULL,
  category VARCHAR(128) NOT NULL,
  manufacturer VARCHAR(255) NOT NULL,
  photo_front VARCHAR(1024) NOT NULL,
  photo_back VARCHAR(1024) NOT NULL,
  location VARCHAR(255) NOT NULL,
  store_id VARCHAR(64) NULL,
  chassis_marker VARCHAR(128) NOT NULL,
  shelf_number INT NULL,
  shelf_zone VARCHAR(16) NOT NULL,
  shelf_layer INT NULL,
  shelf_x DECIMAL(12,3) NOT NULL,
  shelf_y DECIMAL(12,3) NOT NULL,
  shelf_z DECIMAL(12,3) NOT NULL,
  emoji VARCHAR(32) NOT NULL,
  price DECIMAL(10,2) NOT NULL,
  stock INT NOT NULL,
  status TINYINT NOT NULL,
  migrated_at DATETIME NOT NULL,
  PRIMARY KEY (id),
  KEY idx_products_store (store_id),
  KEY idx_products_name (name),
  CONSTRAINT fk_products_store FOREIGN KEY (store_id) REFERENCES stores(id) ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS merchants (
  id VARCHAR(64) NOT NULL,
  username VARCHAR(128) NOT NULL,
  name VARCHAR(128) NOT NULL,
  password_hash CHAR(128) NOT NULL,
  salt VARCHAR(128) NOT NULL,
  status VARCHAR(32) NOT NULL,
  contact VARCHAR(128) NOT NULL,
  phone VARCHAR(64) NOT NULL,
  notes TEXT NOT NULL,
  created_at DATETIME NULL,
  updated_at DATETIME NULL,
  last_login_at DATETIME NULL,
  PRIMARY KEY (id),
  UNIQUE KEY uk_merchants_username (username)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS merchant_stores (
  merchant_id VARCHAR(64) NOT NULL,
  store_id VARCHAR(64) NOT NULL,
  PRIMARY KEY (merchant_id, store_id),
  CONSTRAINT fk_merchant_stores_merchant FOREIGN KEY (merchant_id) REFERENCES merchants(id) ON DELETE CASCADE,
  CONSTRAINT fk_merchant_stores_store FOREIGN KEY (store_id) REFERENCES stores(id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS orders (
  order_id VARCHAR(64) NOT NULL,
  external_order_id VARCHAR(128) NULL,
  total_amount DECIMAL(10,2) NOT NULL,
  payment_mode VARCHAR(32) NOT NULL,
  payment_status VARCHAR(32) NOT NULL,
  order_status VARCHAR(32) NOT NULL,
  source VARCHAR(32) NOT NULL,
  device_id VARCHAR(64) NULL,
  device_sn VARCHAR(128) NULL,
  store_id VARCHAR(64) NULL,
  store_name VARCHAR(128) NULL,
  delivered_units INT NOT NULL,
  delivered_amount DECIMAL(10,2) NOT NULL,
  refund_amount DECIMAL(10,2) NOT NULL,
  refunded_amount DECIMAL(10,2) NOT NULL,
  refund_id VARCHAR(128) NULL,
  refund_status VARCHAR(32) NULL,
  refund_reason TEXT NULL,
  refund_trigger VARCHAR(64) NULL,
  refund_request_id VARCHAR(128) NULL,
  refund_is_partial TINYINT(1) NULL,
  payment_trade_state VARCHAR(32) NULL,
  payment_trade_state_desc VARCHAR(255) NULL,
  paid_source VARCHAR(64) NULL,
  failure_reason TEXT NULL,
  inventory_reservation_error TEXT NULL,
  items_json JSON NOT NULL,
  delivered_items_json JSON NULL,
  refund_items_json JSON NULL,
  refund_history_json JSON NULL,
  created_at DATETIME NULL,
  updated_at DATETIME NULL,
  paid_at DATETIME NULL,
  inventory_reserved_at DATETIME NULL,
  execution_started_at DATETIME NULL,
  placed_at DATETIME NULL,
  completion_reported_at DATETIME NULL,
  completed_at DATETIME NULL,
  failed_at DATETIME NULL,
  last_delivered_at DATETIME NULL,
  partial_stock_restored_at DATETIME NULL,
  stock_restored_at DATETIME NULL,
  refund_requested_at DATETIME NULL,
  refund_last_attempt_at DATETIME NULL,
  partially_refunded_at DATETIME NULL,
  refunded_at DATETIME NULL,
  cancel_requested_at DATETIME NULL,
  payment_last_checked_at DATETIME NULL,
  PRIMARY KEY (order_id),
  KEY idx_orders_store_created (store_id, created_at),
  KEY idx_orders_status (order_status, payment_status),
  KEY idx_orders_external (external_order_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS order_items (
  order_id VARCHAR(64) NOT NULL,
  line_no INT NOT NULL,
  product_id INT NULL,
  name VARCHAR(255) NOT NULL,
  spec VARCHAR(255) NOT NULL,
  qty INT NOT NULL,
  unit_price DECIMAL(10,2) NOT NULL,
  line_total DECIMAL(10,2) NOT NULL,
  payment_menu_item_id INT NULL,
  product_stock INT NULL,
  shelf_layer INT NULL,
  chassis_marker VARCHAR(128) NULL,
  PRIMARY KEY (order_id, line_no),
  KEY idx_order_items_product (product_id),
  CONSTRAINT fk_order_items_order FOREIGN KEY (order_id) REFERENCES orders(order_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS order_refunds (
  order_id VARCHAR(64) NOT NULL,
  sequence_no INT NOT NULL,
  refund_id VARCHAR(128) NULL,
  refund_request_id VARCHAR(128) NULL,
  refund_amount DECIMAL(10,2) NULL,
  refund_status VARCHAR(32) NULL,
  reason TEXT NULL,
  trigger_name VARCHAR(64) NULL,
  refund_items_json JSON NULL,
  completes_refund TINYINT(1) NULL,
  requested_at DATETIME NULL,
  last_attempt_at DATETIME NULL,
  attempt_count INT NULL,
  completed_at DATETIME NULL,
  PRIMARY KEY (order_id, sequence_no),
  KEY idx_order_refunds_id (refund_id),
  CONSTRAINT fk_order_refunds_order FOREIGN KEY (order_id) REFERENCES orders(order_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);

emit(`CREATE TABLE IF NOT EXISTS device_events (
  device_id VARCHAR(64) NOT NULL,
  sequence_no INT NOT NULL,
  type VARCHAR(32) NOT NULL,
  event_at DATETIME NULL,
  store_id VARCHAR(64) NULL,
  store_name VARCHAR(128) NULL,
  location VARCHAR(255) NULL,
  notes TEXT NULL,
  PRIMARY KEY (device_id, sequence_no),
  CONSTRAINT fk_device_events_device FOREIGN KEY (device_id) REFERENCES devices(id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS reserved_orders (
  order_id VARCHAR(64) NOT NULL,
  reserved_at DATETIME NULL,
  expires_at DATETIME NULL,
  payload_json JSON NOT NULL,
  PRIMARY KEY (order_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS completed_orders (
  order_id VARCHAR(64) NOT NULL,
  completed_at DATETIME NULL,
  inventory_restored_at DATETIME NULL,
  payload_json JSON NOT NULL,
  PRIMARY KEY (order_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS inventory_scan_shelves (
  shelf VARCHAR(16) NOT NULL,
  markers_json JSON NOT NULL,
  PRIMARY KEY (shelf)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS inventory_scan_tasks (
  task_key VARCHAR(32) NOT NULL,
  status VARCHAR(32) NULL,
  progress INT NULL,
  payload_json JSON NOT NULL,
  PRIMARY KEY (task_key)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS chassis_settings (
  setting_key VARCHAR(32) NOT NULL,
  navigation_enabled TINYINT(1) NULL,
  auto_return_charge TINYINT(1) NULL,
  place_marker VARCHAR(128) NULL,
  hand_model VARCHAR(32) NULL,
  payload_json JSON NOT NULL,
  PRIMARY KEY (setting_key)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);
emit(`CREATE TABLE IF NOT EXISTS order_callback_outbox (
  order_id VARCHAR(64) NOT NULL,
  event_type VARCHAR(32) NOT NULL,
  completion_token VARCHAR(256) NULL,
  attempts INT NULL,
  next_attempt_at DATETIME NULL,
  payload_json JSON NOT NULL,
  PRIMARY KEY (order_id, event_type)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;`);

emit('START TRANSACTION;');
emit('SET FOREIGN_KEY_CHECKS=0;');
emit('DELETE FROM order_refunds;');
emit('DELETE FROM order_items;');
emit('DELETE FROM orders;');
emit('DELETE FROM merchant_stores;');
emit('DELETE FROM merchants;');
emit('DELETE FROM products;');
emit('DELETE FROM devices;');
emit('DELETE FROM stores;');
emit('DELETE FROM device_events;');
emit('DELETE FROM reserved_orders;');
emit('DELETE FROM completed_orders;');
emit('DELETE FROM inventory_scan_shelves;');
emit('DELETE FROM inventory_scan_tasks;');
emit('DELETE FROM chassis_settings;');
emit('DELETE FROM order_callback_outbox;');
emit('SET FOREIGN_KEY_CHECKS=1;');

for (const row of stores) emit(`INSERT INTO stores (id,name,code,address,contact,phone,status,notes,created_at,updated_at) VALUES (${sqlString(row.id)},${sqlString(row.name)},${row.code ? sqlString(row.code) : 'NULL'},${sqlString(row.address)},${sqlString(row.contact)},${sqlString(row.phone)},${sqlString(row.status || 'active')},${sqlString(row.notes)},${sqlDate(row.createdAt)},${sqlDate(row.updatedAt)});`);
for (const row of devices) emit(`INSERT INTO devices (id,sn,name,model,status,store_id,store_name,location,notes,last_seen_at,created_at,updated_at) VALUES (${sqlString(row.id)},${row.sn ? sqlString(row.sn) : 'NULL'},${sqlString(row.name)},${sqlString(row.model)},${sqlString(row.status || 'in_stock')},${sqlString(row.storeId)},${sqlString(row.storeName)},${sqlString(row.location)},${sqlString(row.notes)},${sqlDate(row.lastSeenAt)},${sqlDate(row.createdAt)},${sqlDate(row.updatedAt)});`);
for (const row of products) emit(`INSERT INTO products (id,name,spec,packaging,category,manufacturer,photo_front,photo_back,location,store_id,chassis_marker,shelf_number,shelf_zone,shelf_layer,shelf_x,shelf_y,shelf_z,emoji,price,stock,status,migrated_at) VALUES (${sqlNumber(row.id)},${sqlString(row.name)},${sqlString(row.spec)},${sqlString(row.packaging)},${sqlString(row.category)},${sqlString(row.manufacturer)},${sqlString(row.photo_front)},${sqlString(row.photo_back)},${sqlString(row.location)},${sqlString(row.storeId || row.store_id)},${sqlString(row.chassis_marker || row.marker_name)},${row.shelf_number == null ? 'NULL' : sqlNumber(row.shelf_number)},${sqlString(row.shelf_zone || row.shelfZone)},${row.shelf_layer == null ? 'NULL' : sqlNumber(row.shelf_layer)},${sqlNumber(row.shelf_x)},${sqlNumber(row.shelf_y)},${sqlNumber(row.shelf_z, 50)},${sqlString(row.emoji || '📦')},${sqlNumber(row.price)},${sqlNumber(row.stock, 100)},${sqlNumber(row.status, 1)},${migratedAt});`);
for (const row of merchants) emit(`INSERT INTO merchants (id,username,name,password_hash,salt,status,contact,phone,notes,created_at,updated_at,last_login_at) VALUES (${sqlString(row.id)},${sqlString(row.username)},${sqlString(row.name)},${sqlString(row.passwordHash)},${sqlString(row.salt)},${sqlString(row.status || 'active')},${sqlString(row.contact)},${sqlString(row.phone)},${sqlString(row.notes)},${sqlDate(row.createdAt)},${sqlDate(row.updatedAt)},${sqlDate(row.lastLoginAt)});`);
for (const row of merchantStores) emit(`INSERT INTO merchant_stores (merchant_id,store_id) VALUES (${sqlString(row.merchantId)},${sqlString(row.storeId)});`);

function sqlJson(value) {
    if (value === null || value === undefined) return 'NULL';
    return sqlString(JSON.stringify(value));
}

for (const order of orders) emit(`INSERT INTO orders (order_id,external_order_id,total_amount,payment_mode,payment_status,order_status,source,device_id,device_sn,store_id,store_name,delivered_units,delivered_amount,refund_amount,refunded_amount,refund_id,refund_status,refund_reason,refund_trigger,refund_request_id,refund_is_partial,payment_trade_state,payment_trade_state_desc,paid_source,failure_reason,inventory_reservation_error,items_json,delivered_items_json,refund_items_json,refund_history_json,created_at,updated_at,paid_at,inventory_reserved_at,execution_started_at,placed_at,completion_reported_at,completed_at,failed_at,last_delivered_at,partial_stock_restored_at,stock_restored_at,refund_requested_at,refund_last_attempt_at,partially_refunded_at,refunded_at,cancel_requested_at,payment_last_checked_at) VALUES (${sqlString(order.orderId)},${sqlString(order.externalOrderId)},${sqlNumber(order.totalAmount)},${sqlString(order.paymentMode || '')},${sqlString(order.paymentStatus || '')},${sqlString(order.status || '')},${sqlString(order.source || '')},${sqlString(order.deviceId)},${sqlString(order.deviceSn)},${sqlString(order.storeId)},${sqlString(order.storeName)},${sqlNumber(order.deliveredUnits)},${sqlNumber(order.deliveredAmount)},${sqlNumber(order.refundAmount)},${sqlNumber(order.refundedAmount)},${sqlString(order.refundId)},${sqlString(order.refundStatus)},${sqlString(order.refundReason)},${sqlString(order.refundTrigger)},${sqlString(order.refundRequestId)},${order.refundIsPartial == null ? 'NULL' : sqlNumber(order.refundIsPartial ? 1 : 0)},${sqlString(order.paymentTradeState)},${sqlString(order.paymentTradeStateDesc)},${sqlString(order.paidSource)},${sqlString(order.failureReason)},${sqlString(order.inventoryReservationError)},${sqlJson(order.items || [])},${sqlJson(order.deliveredItems)},${sqlJson(order.refundItems)},${sqlJson(order.refundHistory)},${sqlDate(order.createdAt)},${sqlDate(order.updatedAt)},${sqlDate(order.paidAt)},${sqlDate(order.inventoryReservedAt)},${sqlDate(order.executionStartedAt)},${sqlDate(order.placedAt)},${sqlDate(order.completionReportedAt)},${sqlDate(order.completedAt)},${sqlDate(order.failedAt)},${sqlDate(order.lastDeliveredAt)},${sqlDate(order.partialStockRestoredAt)},${sqlDate(order.stockRestoredAt)},${sqlDate(order.refundRequestedAt)},${sqlDate(order.refundLastAttemptAt)},${sqlDate(order.partiallyRefundedAt)},${sqlDate(order.refundedAt)},${sqlDate(order.cancelRequestedAt)},${sqlDate(order.paymentLastCheckedAt)});`);
for (const row of orderItems) emit(`INSERT INTO order_items (order_id,line_no,product_id,name,spec,qty,unit_price,line_total,payment_menu_item_id,product_stock,shelf_layer,chassis_marker) VALUES (${sqlString(row.order.orderId)},${row.lineNo},${sqlNumber(row.item.id)},${sqlString(row.item.name)},${sqlString(row.item.spec)},${sqlNumber(row.item.qty)},${sqlNumber(row.item.unitPrice)},${sqlNumber(row.item.lineTotal)},${row.item.paymentMenuItemId == null ? 'NULL' : sqlNumber(row.item.paymentMenuItemId)},${row.item.productStock == null ? 'NULL' : sqlNumber(row.item.productStock)},${row.item.shelf_layer == null ? 'NULL' : sqlNumber(row.item.shelf_layer)},${sqlString(row.item.chassis_marker)});`);
for (const row of orderRefunds) emit(`INSERT INTO order_refunds (order_id,sequence_no,refund_id,refund_request_id,refund_amount,refund_status,reason,trigger_name,refund_items_json,completes_refund,requested_at,last_attempt_at,attempt_count,completed_at) VALUES (${sqlString(row.order.orderId)},${row.sequence},${sqlString(row.refund.refundId)},${sqlString(row.refund.refundRequestId)},${row.refund.refundAmount == null ? 'NULL' : sqlNumber(row.refund.refundAmount)},${sqlString(row.refund.refundStatus)},${sqlString(row.refund.reason)},${sqlString(row.refund.trigger)},${sqlJson(row.refund.refundItems)},${row.refund.completesRefund == null ? 'NULL' : sqlNumber(row.refund.completesRefund ? 1 : 0)},${sqlDate(row.refund.requestedAt)},${sqlDate(row.refund.lastAttemptAt)},${row.refund.attemptCount == null ? 'NULL' : sqlNumber(row.refund.attemptCount)},${sqlDate(row.refund.completedAt)});`);
for (const row of deviceEvents) emit(`INSERT INTO device_events (device_id,sequence_no,type,event_at,store_id,store_name,location,notes) VALUES (${sqlString(row.deviceId)},${row.sequence},${sqlString(row.event.type)},${sqlDate(row.event.at)},${sqlString(row.event.storeId)},${sqlString(row.event.storeName)},${sqlString(row.event.location)},${sqlString(row.event.notes)});`);
for (const [orderId, reservation] of Object.entries(reservedOrders)) emit(`INSERT INTO reserved_orders (order_id,reserved_at,expires_at,payload_json) VALUES (${sqlString(orderId)},${sqlDate(reservation && (reservation.reservedAt || reservation.at || reservation.createdAt))},${sqlDate(reservation && reservation.expiresAt)},${sqlJson(reservation)});`);
for (const [orderId, completion] of Object.entries(completedOrders)) emit(`INSERT INTO completed_orders (order_id,completed_at,inventory_restored_at,payload_json) VALUES (${sqlString(orderId)},${sqlDate(completion && completion.completedAt)},${sqlDate(completion && completion.inventoryRestoredAt)},${sqlJson(completion)});`);
for (const shelf of inventoryShelves) emit(`INSERT INTO inventory_scan_shelves (shelf,markers_json) VALUES (${sqlString(shelf.shelf)},${sqlJson(shelf.markers || [])});`);
if (inventoryTask) emit(`INSERT INTO inventory_scan_tasks (task_key,status,progress,payload_json) VALUES ('last_scan',${sqlString(inventoryTask.status)},${inventoryTask.progress == null ? 'NULL' : sqlNumber(inventoryTask.progress)},${sqlJson(inventoryTask)});`);
if (chassisSettings) emit(`INSERT INTO chassis_settings (setting_key,navigation_enabled,auto_return_charge,place_marker,hand_model,payload_json) VALUES ('default',${chassisSettings.navigation_enabled == null ? 'NULL' : sqlNumber(chassisSettings.navigation_enabled ? 1 : 0)},${chassisSettings.auto_return_charge == null ? 'NULL' : sqlNumber(chassisSettings.auto_return_charge ? 1 : 0)},${sqlString(chassisSettings.place_marker)},${sqlString(chassisSettings.hand_model)},${sqlJson(chassisSettings)});`);
for (const event of outboxEvents) emit(`INSERT INTO order_callback_outbox (order_id,event_type,completion_token,attempts,next_attempt_at,payload_json) VALUES (${sqlString(event.orderId)},${sqlString(event.type || 'completed')},${sqlString(event.completionToken)},${event.attempts == null ? 'NULL' : sqlNumber(event.attempts)},${sqlDate(event.nextAttemptAt)},${sqlJson(event)});`);

emit('COMMIT;');
emit(`-- Imported stores=${stores.length}, devices=${devices.length}, products=${products.length}, merchants=${merchants.length}, merchant_stores=${merchantStores.length}, orders=${orders.length}, order_items=${orderItems.length}, order_refunds=${orderRefunds.length}, device_events=${deviceEvents.length}, reserved_orders=${Object.keys(reservedOrders).length}, completed_orders=${Object.keys(completedOrders).length}, inventory_shelves=${inventoryShelves.length}, outbox_events=${outboxEvents.length}`);
