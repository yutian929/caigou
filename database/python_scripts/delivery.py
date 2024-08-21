import sqlite3

# 连接到数据库（如果不存在会自动创建）
conn = sqlite3.connect('../delivery.db')
c = conn.cursor()

# 创建 deliveries 表（如果不存在）
c.execute('''
CREATE TABLE IF NOT EXISTS deliveries (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    tracking_number TEXT UNIQUE,
    recipient_name TEXT,
    recipient_phone TEXT,
    recipient_address TEXT,
    sender_name TEXT,
    sender_phone TEXT,
    sender_address TEXT,
    courier_company TEXT,     -- 新增列，用于存储快递公司名称
    sent INTEGER DEFAULT 1    -- 标记订单是否已发送, 1是未发送, 0是已发送
)
''')

# 创建 tracking_number_tracker 表（如果不存在）
c.execute('''
CREATE TABLE IF NOT EXISTS tracking_number_tracker (
    current_number INTEGER
)
''')

# 检查是否已经有初始值
c.execute('SELECT COUNT(*) FROM tracking_number_tracker')
if c.fetchone()[0] == 0:
    start_tracking_number = 999999
    c.execute('INSERT INTO tracking_number_tracker (current_number) VALUES (?)', (start_tracking_number,))

# 提交更改并关闭连接
conn.commit()
conn.close()

print("Database 'delivery.db' initialized with table 'deliveries' including 'courier_company' and starting tracking number set.")
