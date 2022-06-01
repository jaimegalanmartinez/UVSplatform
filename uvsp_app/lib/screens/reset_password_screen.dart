import 'package:flutter/material.dart';
import 'package:uvsp_app/forms/form_reset_password.dart';

class ResetPasswordScreen extends StatefulWidget {
  const ResetPasswordScreen({Key? key}) : super(key: key);

  @override
  State<ResetPasswordScreen> createState() => _ResetPasswordScreenState();
}

class _ResetPasswordScreenState extends State<ResetPasswordScreen> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: const Text('Reset password'),
        ),
        backgroundColor: Colors.white,
        body: Container(
            margin: const EdgeInsets.all(16.0),
            height: double.infinity,
            width: double.infinity,
            decoration: const BoxDecoration(color: Colors.white),
            child: const FormResetPassword()));
  }

}
