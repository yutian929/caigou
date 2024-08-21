import sqlite3

# 连接到 SQLite 数据库
conn = sqlite3.connect('../express.db')

# 创建一个游标对象
cursor = conn.cursor()

# 清空表中的数据
cursor.execute('DELETE FROM express_info')

# 重置自增计数器
cursor.execute('DELETE FROM sqlite_sequence WHERE name="express_info"')

# 创建一个只包含取件码的列表
data_to_insert = [
    (None, '1-1-1', None, None, None, None, None, None),
    ('000001', '1-1-2', '张雨田', '18012964897', '广东省深圳市', '李昌', '18123456789', '北京市朝阳区'),  # 插入一条完整的数据
    (None, '1-1-3', None, None, None, None, None, None),
    (None, '1-1-4', None, None, None, None, None, None),
    (None, '1-1-5', None, None, None, None, None, None),
]

# 插入只包含取件码的数据
cursor.executemany('''
    INSERT INTO express_info (tracking_number, pickup_code, recipient_name, recipient_phone, recipient_address, sender_name, sender_phone, sender_address)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
''', data_to_insert)

# 提交事务
conn.commit()

print('插入的行数:', cursor.rowcount)

# 关闭连接
conn.close()
