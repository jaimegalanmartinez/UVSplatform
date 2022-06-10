import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:uvsp_app/forms/form_validators.dart';
import 'package:firebase_auth/firebase_auth.dart';

import '../utils/constants.dart';



class FormResetPassword extends StatefulWidget {
  const FormResetPassword({Key? key}) : super(key: key);

  @override
  State<FormResetPassword> createState() => _FormResetPasswordState();
}

class _FormResetPasswordState extends State<FormResetPassword> {
  final _formResetPasswordKey = GlobalKey<FormState>();
  final TextEditingController _emailController = TextEditingController();
  late String _emailToSendReset;

  Future sendResetPasswordEmail(String email) async {
    try{
      if (kDebugMode) {
        print('Email to send password reset: $email');
      }
      await FirebaseAuth.instance.sendPasswordResetEmail(email: email).then((value) {
        ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('Password reset email sent')));
      });

    }on FirebaseAuthException catch(err){
      ScaffoldMessenger.of(context).showSnackBar(
           SnackBar(content: Text(err.message ?? 'Unknown error')));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Form(
      key: _formResetPasswordKey,
      child: Column(crossAxisAlignment: CrossAxisAlignment.center, children: [
        const SizedBox(
          height: 20.0,
        ),
        TextFormField(
          controller: _emailController,
          style: const TextStyle(color: Colors.black),
          cursorColor: const Color.fromRGBO(58, 66, 86, 1.0),
          decoration: InputDecoration(
            filled: true,
            fillColor: Colors.white,
            icon: const Padding(
              padding: EdgeInsets.all(8.0),
              child: Icon(Icons.email, color: Colors.white,),
            ),
            border: OutlineInputBorder(
              borderSide: const BorderSide(),
              borderRadius: BorderRadius.circular(20.0),
            ),
            hintText: 'Type email to send password reset',
              hintStyle: const TextStyle(color: Colors.grey),
              errorMaxLines: 3,
              errorStyle: const TextStyle(color: Colors.deepOrangeAccent)
          ),
          validator: validateEmail,
          onSaved: (value) => _emailToSendReset = value!,
        ),
        const SizedBox(
          height: 60.0,
        ),
        ElevatedButton(
            style: ButtonStyle(
                backgroundColor: MaterialStateProperty.all<Color>(Colors.white),
                minimumSize: MaterialStateProperty.all<Size>(const Size(160, 64))),
            onPressed: () {
              //Validate returns true if the form is valid, or false otherwise.
              if (_formResetPasswordKey.currentState!.validate()) {
                _formResetPasswordKey.currentState?.save();

                sendResetPasswordEmail(_emailToSendReset.trim());
              }
            },
            child: const Text(
              'Send password\n reset',
              textAlign: TextAlign.center,
              style: kLabelSendStyle,
            )),
      ]),
    );
  }
}

