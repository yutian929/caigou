import sqlite3

# 连接到 SQLite 数据库（如果数据库不存在，会自动创建）
conn = sqlite3.connect('../express.db')

# 创建一个游标对象
cursor = conn.cursor()

# 创建快递信息表
cursor.execute('''
    CREATE TABLE IF NOT EXISTS express_info (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        tracking_number TEXT,
        pickup_code TEXT NOT NULL UNIQUE,
        recipient_name TEXT,
        recipient_phone TEXT,
        recipient_address TEXT,
        sender_name TEXT,
        sender_phone TEXT,
        sender_address TEXT
    )
''')

# 插入初始取件码（1-1-1 到 1-1-5）
initial_codes = [f"1-1-{i}" for i in range(1, 6)]
for code in initial_codes:
    cursor.execute('''
        INSERT OR IGNORE INTO express_info (pickup_code)
        VALUES (?)
    ''', (code,))
# 提交事务
conn.commit()

# 关闭连接
conn.close()
