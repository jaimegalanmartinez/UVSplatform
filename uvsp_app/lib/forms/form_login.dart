import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/foundation.dart';
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
  final _auth = FirebaseAuth.instance;
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
            style: const TextStyle(color:Colors.black),
            cursorColor: const Color.fromRGBO(58, 66, 86, 1.0),
            decoration: InputDecoration(
              filled: true,
              fillColor: Colors.white,
              icon: const Padding(
                padding: EdgeInsets.all(8.0),
                child: Icon(Icons.email, color: Colors.white,),
              ),
              border: OutlineInputBorder(
                borderSide: const BorderSide(color: Colors.white),
                borderRadius: BorderRadius.circular(20.0),
              ),
              hintText: 'Email',
              hintStyle: const TextStyle(color: Colors.grey),
              errorStyle: const TextStyle(color: Colors.deepOrangeAccent),
            ),
            validator: validateEmail,
            onSaved: (value) => _userEmail = value!,
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
                child: Icon(Icons.lock, color: Colors.white,),
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
            height: 36.0,
          ),
          ElevatedButton(
            style: ButtonStyle(
                backgroundColor: MaterialStateProperty.all<Color>(Colors.white),
                minimumSize: MaterialStateProperty.all<Size>(const Size(140, 40))),
            onPressed: () async {
              //Validate returns true if the form is valid, or false otherwise.
              if (_formKey.currentState!.validate()) {
                _formKey.currentState?.save();
                if (kDebugMode) {
                  print('User email: $_userEmail with password: $_userPassword');
                }
                final newUserCredential = await _auth.signInWithEmailAndPassword(email: _userEmail.trim(), password: _userPassword.trim()).then((value) {
                  Navigator.push(context, MaterialPageRoute(builder: (context) => const HomeScreen(title: 'UVS Platform')));
                }).onError((FirebaseAuthException error, stackTrace) {
                  if (kDebugMode) {
                    print("Error ${error.toString()}");
                  }
                  showDialog(context: context, builder: (context) => AlertDialog(
                    title: const Text('Login failed'),
                    content: Text(error.message ?? 'Unknown error'),
                  ));
                });
              } //end if validate form
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