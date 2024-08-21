import sqlite3

# 连接到 SQLite 数据库
conn = sqlite3.connect('../express.db')

# 创建一个游标对象
cursor = conn.cursor()

# 创建一个包含多行数据的列表
data_to_insert = [
    ('000001', '1-1-6', '张雨田', '18012964897', '广东省深圳市', '李昌', '18123456789', '北京市朝阳区'),
]

# 插入数据
cursor.executemany('''
    INSERT INTO express_info (tracking_number, pickup_code, recipient_name, recipient_phone, recipient_address, sender_name, sender_phone, sender_address)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
''', data_to_insert)

# 提交事务
conn.commit()

print('插入的行数:', cursor.rowcount)

# 关闭连接
conn.close()
