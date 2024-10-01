#!/bin/bash

USER_HOME=/home/hustlyrm

# @brief 检查并创建文件
# @param $1 要检查并创建文件的绝对路径，包含文件名称
# @warn 遇到错误会记录日志并且退出脚本
CheckCreateFile() {
	if [ -z $1 ]; then
		echo "[x] 请提供文件名"
		exit 1
	fi

	echo "[#] 正在检查文件: $1"
	if [ -f $1 ]; then
		echo "[!] 文件已存在: $1"
		return
	fi

	echo "[#] 正在尝试创建文件: $1"
	touch $1
	if [ ! -f $1 ]; then
		echo "[x] 创建文件失败"
		exit 1
	fi
	echo "[#] 创建文件成功"
}

echo "([#] INFO) ([!] WARN) ([x] ERROR)"

echo "[#] 正在检查管理员权限"
if [ $(id -u) -ne 0 ]; then
	echo "[x] 请使用管理员权限运行此脚本，当前无法继续执行"
	exit 1
fi

echo "[#] 正在检查文件：/etc/systemd/system/rc-local.service"
if [ ! -f /etc/systemd/system/rc-local.service ]; then
	echo "[!] 未找到文件：/etc/systemd/system/rc-local.service"
	echo "[#] 正在尝试复制文件：cp /usr/lib/systemd/system/rc-local.service /etc/systemd/system/"
	cp /usr/lib/systemd/system/rc-local.service /etc/systemd/system/
	if [ ! -f /etc/systemd/system/rc-local.service ]; then
		echo "[x] 复制文件失败，当前无法继续执行"
		exit 1
	fi
	echo "[#] 复制文件成功"
fi

CheckCreateFile /etc/rc.local
FILE_CONTENT_RC_LOCAL="#!/bin/bash\n\nbash ${USER_HOME}/autorun.d/.autorun &\n\nexit 0"
echo "[#] 正在向文件 /etc/rc.local 覆写内容： ${FILE_CONTENT_RC_LOCAL}"
echo -e ${FILE_CONTENT_RC_LOCAL} > /etc/rc.local
chmod +x /etc/rc.local

echo "[#] 正在检查目录：${USER_HOME}/autorun.d"
if [ ! -d ${USER_HOME}/autorun.d ]; then
	echo "[!] 未找到目录：${USER_HOME}/autorun.d"
	echo "[#] 正在尝试创建目录 mkdir ${USER_HOME}/autorun.d"
	mkdir ${USER_HOME}/autorun.d
	if [ ! -d ${USER_HOME}/autorun.d ]; then
		echo "[x] 创建目录失败，当前无法继续执行"
		exit 1
	fi
	echo "[#] 创建目录成功"
fi
echo "[#] 进入目录：${USER_HOME}/autorun.d"
cd ${USER_HOME}/autorun.d


CheckCreateFile .autorun
echo "[#] 正在向文件 .autorun 覆写内容"

cat << AUTORUN_EOF > .autorun
#!/bin/bash
echo "([#] INFO) ([!] WARN) ([x] ERROR)"
echo "[#] autorun: ${USER_HOME}/autorun.d"
cd ${USER_HOME}/autorun.d
for file in \$(ls *.enabled); do
	echo "[#] running enabled file: ${USER_HOME}/autorun.d/\${file}"
	bash ${USER_HOME}/autorun.d/\${file} &
done
exit 0
AUTORUN_EOF

chmod +x .autorun

echo "[#] 正在启动 rc-local.service"
systemctl start rc-local

echo "[#] 正在打印 rc-local 服务信息，如果卡住，请自行退出"
service rc-local status

echo "[#] 脚本执行结束，如果有问题请在此仓库下提 issue"

exit 0
