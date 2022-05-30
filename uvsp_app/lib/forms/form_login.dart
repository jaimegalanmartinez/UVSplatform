import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

import 'package:uvsp_app/forms/form_validators.dart';
import 'package:uvsp_app/screens/home_screen.dart';
import 'package:uvsp_app/utils/constants.dart';

class LoginForm extends StatefulWidget {
  const LoginForm({Key? key}) : super(key: key);

  @override
  State<LoginForm> createState() => _LoginFormState();
}

class _LoginFormState extends State<LoginForm> {
  final _formKey = GlobalKey<FormState>();

  late String _userEmail;
  late String _userPassword;

  @override
  Widget build(BuildContext context) {
    return Form(
      key: _formKey,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          TextFormField(
            decoration: InputDecoration(
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(Icons.email),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Email',
            ),
            validator: validateEmail,
            onSaved: (value) => _userEmail = value!,
          ),
          const SizedBox(
            height: 12.0,
          ),
          TextFormField(
            decoration: InputDecoration(
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(Icons.lock),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Password',
              errorMaxLines: 3,
            ),
            validator: validatePassword,
            obscureText: true,
            onSaved: (value) => _userPassword = value!,
          ),
          const SizedBox(
            height: 36.0,
          ),
          ElevatedButton(
            style: ButtonStyle(
                backgroundColor: MaterialStateProperty.all<Color>(Colors.blue),
                minimumSize: MaterialStateProperty.all<Size>(const Size(140, 40))),
            onPressed: () {
              //Validate returns true if the form is valid, or false otherwise.
              if (_formKey.currentState!.validate()) {
                _formKey.currentState?.save();
                ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('Processing login data')));
                print('User email: $_userEmail with password: $_userPassword');
                //Just for example to go to Home Screen
                if (_userEmail == 'test@gmail.com' &&
                    _userPassword == '1aQ!test') {
                  Navigator.pushAndRemoveUntil(
                      context,
                      MaterialPageRoute(
                          builder: (BuildContext context) =>
                              const MyHomePage(title: 'UVS Platform')),
                          (Route<dynamic> route) => false);
                }
              }
            },
            child: const Text(
              'Login',
              style: kLabelLoginSignUpStyle,
            ),
          )
        ],
      ),
    );
  }
}

/*Future signIn() async {
  await FirebaseAuth.instance.signInWithEmailAndPassword(
      email: emailController.text.trim(),
      password: passwordController.text.trim());
}*/