import 'package:flutter/material.dart';
import 'package:uvsp_app/screens/login_screen.dart';
import 'package:uvsp_app/forms/form_signup.dart';
import 'package:uvsp_app/utils/constants.dart';


class SignUpScreen extends StatefulWidget {
  const SignUpScreen({Key? key}) : super(key: key);

  @override
  State<SignUpScreen> createState() => _SignUpScreenState();
}

class _SignUpScreenState extends State<SignUpScreen> {
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
              'Sign Up',
              style: kLabelSignInStyle,
            ),
            const SignUpForm(),
            TextButton(
                onPressed: () {
                  Navigator.push(
                      context,
                      MaterialPageRoute(
                          builder: (BuildContext context) => const LoginScreen(
                              )));
                },
                child: const Text(
                  'Already have an account? Log in',
                  style: kLabelForgotPasswdAlreadyAcc,
                ))
          ],
        ),
      ),
    );
  }
}
