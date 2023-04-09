arduino ide で以下のエラーが出たので理由を教えて。 multiple definition of 
Arduino IDEで"multiple definition of"というエラーが出る場合、通常は同じ名前の関数、変数、またはオブジェクトが複数回定義されていることが原因です。これは、コードの中で同じ名前を持つ複数の宣言がある場合に起こることがあります。

このエラーを解決するには、次のいずれかの方法を試すことができます。

ヘッダファイルの重複を避けるために、必要なヘッダファイルを1つのファイルにまとめることができます。

同じ名前を持つ関数、変数、またはオブジェクトの宣言が複数ある場合、重複した宣言を削除することができます。

グローバル変数を使用する場合、変数を複数のファイルで宣言しないように注意する必要があります。

ライブラリを使用する場合、異なるライブラリが同じ名前の関数や変数を定義していないか確認してください。

これらの解決策がうまく機能しない場合は、コードの一部を見直したり、ライブラリのバージョンを変更したりする必要があるかもしれません。




Shimarisu Ashime
arduino ideで以下のエラーが出た理由を教えて。sketch\CugoArduinoMiddle.cpp.o (symbol from plugin): In function `cugo_motor_controllers':
(.text+0x0): multiple definition of `cugo_motor_controllers'
sketch\CuGoArduinoMiddleUserProgramming.ino.cpp.o (symbol from plugin):(.text+0x0): first defined here
collect2.exe: error: ld returned 1 exit status
exit status 1
ボードArduino Unoに対するコンパイル時にエラーが発生しました。
このエラーは、複数のファイルで同じ名前の関数または変数が定義されていることを示しています。具体的には、"cugo_motor_controllers"という名前のシンボルが、2つのオブジェクトファイル（"CugoArduinoMiddle.cpp.o"と"CuGoArduinoMiddleUserProgramming.ino.cpp.o"）で定義されているため、リンカーによってエラーが発生しました。

この問題を解決するには、2つのファイルで同じ名前のシンボルを定義しないようにする必要があります。例えば、"CugoArduinoMiddle.cpp"ファイルに定義された"cugo_motor_controllers"関数の定義を削除するか、"CuGoArduinoMiddleUserProgramming.ino.cpp"ファイルに定義された"cugo_motor_controllers"関数の定義を削除することができます。

また、このエラーが発生する可能性がある他の原因として、複数のライブラリが同じ名前の関数または変数を定義している場合があります。この場合、ライブラリのバージョンを変更することで解決することができます。




Shimarisu Ashime
CuGoArduinoMiddleUserProgramming:29:49: error: redefinition of 'MotorController cugo_motor_controllers [2]'
 MotorController cugo_motor_controllers[MOTOR_NUM];
                                                 ^
In file included from C:\Users\tamia\Desktop\cuborex\cugo-arduino-middleuser-programming\CuGoArduinoMiddleUserProgramming\CuGoArduinoMiddleUserProgramming.ino:23:0:
C:\Users\tamia\Desktop\cuborex\cugo-arduino-middleuser-programming\CuGoArduinoMiddleUserProgramming\CugoArduinoMiddle.h:111:17: note: 'MotorController cugo_motor_controllers [2]' previously declared here
 MotorController cugo_motor_controllers[MOTOR_NUM];
                 ^~~~~~~~~~~~~~~~~~~~~~
exit status 1
redefinition of 'MotorController cugo_motor_controllers [2]'
このエラーは、同じ名前の変数が2回定義されていることを示しています。具体的には、"MotorController cugo_motor_controllers [2]"という変数が、"CuGoArduinoMiddleUserProgramming.ino"ファイルと"CugoArduinoMiddle.h"ファイルの両方で定義されているため、コンパイラがエラーを報告しています。

この問題を解決するには、1つの定義に変数をまとめる必要があります。例えば、"CuGoArduinoMiddleUserProgramming.ino"ファイル内で変数を定義し、"CugoArduinoMiddle.h"ファイルから変数の宣言を削除することができます。

また、変数の宣言には、"extern"キーワードを使用してグローバルに宣言することもできます。この場合、変数を定義するファイルでのみ初期化する必要があります。例えば、"CugoArduinoMiddle.cpp"ファイル内で変数を定義し、"CugoArduinoMiddle.h"ファイルで変数を宣言することができます。

最後に、このエラーが発生する他の原因として、複数のヘッダファイルが同じ変数を定義している場合があります。この場合、各ヘッダファイルでの変数の宣言を削除するか、1つのヘッダファイルに変数の宣言をまとめることができます。




Shimarisu Ashime
CugoArduinoMiddle.cpp:33:50: error: conflicting declaration 'MotorController cugo_motor_controllers [0]'
 MotorController cugo_motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, CUGO_L_reverse);
                                                  ^
In file included from C:\Users\tamia\Desktop\cuborex\cugo-arduino-middleuser-programming\CuGoArduinoMiddleUserProgramming\CugoArduinoMiddle.cpp:1:0:
C:\Users\tamia\Desktop\cuborex\cugo-arduino-middleuser-programming\CuGoArduinoMiddleUserProgramming\CugoArduinoMiddle.h:111:24: note: previous declaration as 'MotorController cugo_motor_controllers [2]'
 extern MotorController cugo_motor_controllers[MOTOR_NUM];
                        ^~~~~~~~~~~~~~~~~~~~~~
CugoArduinoMiddle.cpp:34:51: error: conflicting declaration 'MotorController cugo_motor_controllers [1]'
 MotorController cugo_motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, CUGO_R_reverse);
                                                   ^
In file included from C:\Users\tamia\Desktop\cuborex\cugo-arduino-middleuser-programming\CuGoArduinoMiddleUserProgramming\CugoArduinoMiddle.cpp:1:0:
C:\Users\tamia\Desktop\cuborex\cugo-arduino-middleuser-programming\CuGoArduinoMiddleUserProgramming\CugoArduinoMiddle.h:111:24: note: previous declaration as 'MotorController cugo_motor_controllers [2]'
 extern MotorController cugo_motor_controllers[MOTOR_NUM];
                        ^~~~~~~~~~~~~~~~~~~~~~
exit status 1
conflicting declaration 'MotorController cugo_motor_controllers [0]'
このエラーは、"CugoArduinoMiddle.h"ファイルで"MotorController cugo_motor_controllers[MOTOR_NUM]"という変数が2つの要素を持つ配列として宣言されているにも関わらず、"CugoArduinoMiddle.cpp"ファイルでその配列の要素数が0または1になっていることを示しています。このため、コンパイラがエラーを報告しています。

この問題を解決するには、"CugoArduinoMiddle.cpp"ファイルで変数の宣言を修正する必要があります。例えば、次のように修正することができます。

cpp
Copy code
#include "CugoArduinoMiddle.h"

MotorController cugo_motor_controllers[MOTOR_NUM] = {
  MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, CUGO_L_reverse),
  MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, CUGO_R_reverse)
};
このコードでは、配列の要素数がMOTOR_NUMに設定されていることに対応して、2つの要素を持つ配列として宣言されています。また、各要素にはMotorControllerのコンストラクタを使用して初期化されています。

このように修正することで、"CugoArduinoMiddle.cpp"ファイルで定義された変数が"CugoArduinoMiddle.h"ファイルで宣言された変数と一致するようになり、エラーが解決されるはずです。