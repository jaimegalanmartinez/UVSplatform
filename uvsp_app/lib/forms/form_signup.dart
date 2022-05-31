import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

import 'package:uvsp_app/forms/form_validators.dart';
import 'package:uvsp_app/screens/home_screen.dart';
import 'package:uvsp_app/screens/login_screen.dart';

import 'package:uvsp_app/utils/constants.dart';


class SignUpForm extends StatefulWidget {
  const SignUpForm({Key? key}) : super(key: key);

  @override
  State<SignUpForm> createState() => _SignUpFormState();
}

class _SignUpFormState extends State<SignUpForm> {
  final _formKey = GlobalKey<FormState>();
  final _auth = FirebaseAuth.instance;
  final TextEditingController _passwordController = TextEditingController();
  final TextEditingController _confirmPassController = TextEditingController();
  late String _userEmail;
  late String _username;
  late String _userPassword;
  late String _confirmPassword;

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
            controller: _passwordController,
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
            height: 12.0,
          ),
          TextFormField(
            controller: _confirmPassController,
            decoration: InputDecoration(
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(Icons.lock),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Confirm password',
              errorMaxLines: 3,
            ),
            validator: (value) {
              if(value!.isEmpty){
                return 'Please enter re-password.';

              }else if(_confirmPassController.text != _passwordController.text){
                return "Passwords do not match";

              }else{
                return null;
              }
            },
            obscureText: true,
            onSaved: (value) => _confirmPassword = value!,
          ),
          const SizedBox(
            height: 12.0,
          ),
          ElevatedButton(
            style: ButtonStyle(
                backgroundColor: MaterialStateProperty.all<Color>(Colors.blue),
                minimumSize: MaterialStateProperty.all<Size>(const Size(140, 40))),
            onPressed: () async {
              //Validate returns true if the form is valid, or false otherwise.

              if (_formKey.currentState!.validate()) {
                _formKey.currentState?.save();
                if (kDebugMode) {
                  print('User email: $_userEmail with password: $_userPassword');
                }
                  final newUserCredential = await _auth.createUserWithEmailAndPassword(email: _userEmail.trim(), password: _userPassword.trim()).then((value) {
                    ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(content: Text('Successfully registered. You can login now')));
                    Navigator.push(context, MaterialPageRoute(builder: (context) => const LoginScreen()));
                  }).onError((FirebaseAuthException error, stackTrace) {
                     if (kDebugMode) {
                       print("Error ${error.toString()}");
                     }
                     showDialog(context: context, builder: (context) => AlertDialog(
                       title: const Text('Registration failed'),
                       content: Text(error.message ?? 'Unknown error'),
                     ));
                  });
              } //end if validate form
            },
            child: const Text(
              'Sign up',
              style: kLabelLoginSignUpStyle,
            ),
          )
        ],
      ),
    );
  }
}