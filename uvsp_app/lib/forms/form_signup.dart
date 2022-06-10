import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

import 'package:uvsp_app/forms/form_validators.dart';
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

  @override
  Widget build(BuildContext context) {
    return Form(
      key: _formKey,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          TextFormField(
            style: const TextStyle(color: Colors.black),
            cursorColor: const Color.fromRGBO(58, 66, 86, 1.0),
            decoration: InputDecoration(
                filled: true,
                fillColor: Colors.white,
                icon: const Padding(
                  padding: EdgeInsets.all(8.0),
                  child: Icon(Icons.person, color: Colors.white),
                ),
                border: OutlineInputBorder(
                  borderSide: const BorderSide(),
                  borderRadius: BorderRadius.circular(20.0),
                ),
                hintText: 'Username',
                hintStyle: const TextStyle(color: Colors.grey),
                errorMaxLines: 3,
                errorStyle: const TextStyle(color: Colors.deepOrangeAccent)),
            validator: validateUsername,
            onSaved: (value) => _username = value!,
          ),
          const SizedBox(
            height: 12.0,
          ),
          TextFormField(
            style: const TextStyle(color: Colors.black),
            cursorColor: const Color.fromRGBO(58, 66, 86, 1.0),
            decoration: InputDecoration(
              filled: true,
              fillColor: Colors.white,
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(
                  Icons.email,
                  color: Colors.white,
                ),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Email',
              hintStyle: const TextStyle(color: Colors.grey),
              errorMaxLines: 3,
              errorStyle: const TextStyle(color: Colors.deepOrangeAccent),
            ),
            validator: validateEmail,
            onSaved: (value) => _userEmail = value!,
          ),
          const SizedBox(
            height: 12.0,
          ),
          TextFormField(
            controller: _passwordController,
            style: const TextStyle(color: Colors.black),
            cursorColor: const Color.fromRGBO(58, 66, 86, 1.0),
            decoration: InputDecoration(
              filled: true,
              fillColor: Colors.white,
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(
                  Icons.lock,
                  color: Colors.white,
                ),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(color: Colors.white),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Password',
              hintStyle: const TextStyle(color: Colors.grey),
              errorMaxLines: 3,
              errorStyle: const TextStyle(color: Colors.deepOrangeAccent),
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
            style: const TextStyle(color: Colors.black),
            cursorColor: const Color.fromRGBO(58, 66, 86, 1.0),
            decoration: InputDecoration(
              filled: true,
              fillColor: Colors.white,
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(Icons.lock, color: Colors.white),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Confirm password',
              hintStyle: const TextStyle(color: Colors.grey),
              errorMaxLines: 3,
              errorStyle: const TextStyle(color: Colors.deepOrangeAccent),
            ),
            validator: (value) {
              if (value!.isEmpty) {
                return 'Please enter re-password.';
              } else if (_confirmPassController.text !=
                  _passwordController.text) {
                return "Passwords do not match";
              } else {
                return null;
              }
            },
            obscureText: true,
          ),
          const SizedBox(
            height: 26.0,
          ),
          ElevatedButton(
            style: ButtonStyle(
                backgroundColor: MaterialStateProperty.all<Color>(Colors.white),
                minimumSize:
                    MaterialStateProperty.all<Size>(const Size(140, 40))),
            onPressed: () async {
              //Validate returns true if the form is valid, or false otherwise.

              if (_formKey.currentState!.validate()) {
                _formKey.currentState?.save();
                if (kDebugMode) {
                  print(
                      'User email: $_userEmail with password: $_userPassword');
                }
                await _auth
                    .createUserWithEmailAndPassword(
                        email: _userEmail.trim(),
                        password: _userPassword.trim())
                    .then((newUserCredential) {
                  newUserCredential.user?.updateDisplayName(_username);
                  ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
                      content:
                          Text('Successfully registered. You can login now')));
                  Navigator.push(
                      context,
                      MaterialPageRoute(
                          builder: (context) => const LoginScreen()));
                }).onError((FirebaseAuthException error, stackTrace) {
                  if (kDebugMode) {
                    print("Error ${error.toString()}");
                  }
                  showDialog(
                      context: context,
                      builder: (context) => AlertDialog(
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
