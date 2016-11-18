find . -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "*.m" > cscope.files
uncrustify -c /Users/yinghuang/development/hy_code/tools/BeautifyCode/hy_c_c++_for_osx.cfg -F cscope.files --no-backup
