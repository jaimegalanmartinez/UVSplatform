import 'package:flutter/material.dart';
import 'package:firebase_core/firebase_core.dart';
import 'firebase_options.dart';
import 'package:uvsp_app/screens/login_screen.dart';
import 'package:uvsp_app/screens/signup_screen.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp(options: DefaultFirebaseOptions.currentPlatform);
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'UVS Platform',
      theme: ThemeData(
        // This is the theme of your application.
        primarySwatch: Colors.grey,
        appBarTheme: const AppBarTheme(
          //backgroundColor: Color.fromRGBO(94, 106, 135, 1.0),
          backgroundColor: Color.fromRGBO(71, 82, 107, 1.0),
          foregroundColor: Colors.white,
        )
      ),
      home: const LoginScreen(),
    );
  }
}

