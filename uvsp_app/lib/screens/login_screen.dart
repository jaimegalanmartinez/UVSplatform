import 'package:flutter/material.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:uvsp_app/forms/form_login.dart';
import 'package:uvsp_app/screens/reset_password_screen.dart';
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
      backgroundColor: const Color.fromRGBO(58, 66, 86, 1.0),
      body: Container(
        margin: const EdgeInsets.all(16.0),
        height: double.infinity,
        width: double.infinity,
        decoration: const BoxDecoration(color: Color.fromRGBO(58, 66, 86, 1.0)),
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
                    style: kLabelForgotPasswdAlreadyAcc,
                  )),
              const SizedBox(height: 10,),
              GestureDetector(
                  onTap: () {
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (BuildContext context) =>
                            const ResetPasswordScreen()));
                  },
                  child: const Text(
                    'Forgot password?',
                    style: TextStyle(decoration: TextDecoration.underline,
                    fontWeight: FontWeight.bold, fontSize: 16.0, color: Colors.white),
                  ))
            ],)
          ],
        ),
      ),
    );
  }
}

