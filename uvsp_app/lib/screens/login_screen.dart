import 'package:flutter/material.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:uvsp_app/screens/home_screen.dart';
import 'package:uvsp_app/forms/form_login.dart';
import 'package:uvsp_app/screens/signup_screen.dart';
import 'package:uvsp_app/utils/constants.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({Key? key}) : super(key: key);

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      //Container by default with white colour background, you can change the color or use a gradient.
      backgroundColor: Colors.white,
      body: Container(
        margin: const EdgeInsets.all(16.0),
        height: double.infinity,
        width: double.infinity,
        decoration: const BoxDecoration(color: Colors.white),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            //For example your App's Name
            const Text(
              'UVS Platform',
              style: kLabelSignInStyle,
            ),
            const LoginForm(),
            Column( children: [
              TextButton(
                  onPressed: () {
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (BuildContext context) => const SignUpScreen(
                            )));
                  },
                  child: const Text(
                    'Don\'t have an account? Sign up',
                    style: kLabelForgotPassword,
                  )),
              const SizedBox(height: 10,),
              TextButton(
                  onPressed: () {
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (BuildContext context) =>
                            const MyHomePage(title: 'Home',)));
                  },
                  child: const Text(
                    'Forgot password?',
                    style: kLabelForgotPassword,
                  ))
            ],)
          ],
        ),
      ),
    );
  }
}

